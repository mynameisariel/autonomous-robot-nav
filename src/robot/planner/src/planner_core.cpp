#include "planner_core.hpp"

namespace robot
{

  PlannerCore::PlannerCore(const rclcpp::Logger &logger)
      : logger_(logger) {}

  nav_msgs::msg::Path PlannerCore::computePath(
      const nav_msgs::msg::OccupancyGrid &map,
      const geometry_msgs::msg::Pose &start,
      const geometry_msgs::msg::PointStamped &goal)
  {
    nav_msgs::msg::Path result;
    result.header.frame_id = "map";

    // ── 1. Convert world coordinates → grid cells ────────────────────────────
    CellIndex start_cell, goal_cell;

    if (!worldToGrid(map, start.position.x, start.position.y, start_cell))
    {
      RCLCPP_WARN(logger_, "Start position (%.2f, %.2f) is outside the map.",
                  start.position.x, start.position.y);
      return result;
    }

    if (!worldToGrid(map, goal.point.x, goal.point.y, goal_cell))
    {
      RCLCPP_WARN(logger_, "Goal position (%.2f, %.2f) is outside the map.",
                  goal.point.x, goal.point.y);
      return result;
    }

    if (!isTraversable(map, goal_cell))
    {
      RCLCPP_WARN(logger_, "Goal cell (%d, %d) is occupied or unknown.", goal_cell.x, goal_cell.y);
      return result;
    }

    // ── 2. A* data structures ────────────────────────────────────────────────
    //
    // g_score[cell]  : best known cost from start → cell
    // came_from[cell]: parent cell on the best known path
    //
    // The open set is a min-heap keyed on f = g + h.
    // We use a "lazy deletion" pattern: nodes are pushed with their current
    // f-score, and stale entries (where g_score has since improved) are
    // discarded when popped.

    using PQ = std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>>;

    std::unordered_map<CellIndex, double, CellIndexHash> g_score;
    std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
    PQ open_set;

    g_score[start_cell] = 0.0;
    open_set.emplace(start_cell, heuristic(start_cell, goal_cell));

    bool found = false;

    // ── 3. Main A* loop ──────────────────────────────────────────────────────
    while (!open_set.empty())
    {
      AStarNode current = open_set.top();
      open_set.pop();

      // Goal check
      if (current.index == goal_cell)
      {
        found = true;
        break;
      }

      double current_g = g_score.count(current.index) ? g_score[current.index]
                                                      : std::numeric_limits<double>::infinity();

      // Lazy deletion: skip if this entry is stale
      double expected_f = current_g + heuristic(current.index, goal_cell);
      if (current.f_score > expected_f + 1e-9)
      {
        continue;
      }

      // Expand neighbours
      for (const CellIndex &neighbour : getNeighbours(current.index))
      {
        // Skip cells that are out of bounds or occupied
        if (!isTraversable(map, neighbour))
        {
          continue;
        }

        // Cost of moving to the neighbour (diagonal costs √2, cardinal costs 1)
        int dx = std::abs(neighbour.x - current.index.x);
        int dy = std::abs(neighbour.y - current.index.y);
        double step_cost = (dx + dy == 2) ? std::sqrt(2.0) : 1.0;

        double tentative_g = current_g + step_cost;

        // Check if this path to neighbour is better than any previous one
        bool in_open = g_score.count(neighbour) > 0;
        if (!in_open || tentative_g < g_score[neighbour])
        {
          g_score[neighbour] = tentative_g;
          came_from[neighbour] = current.index;
          double f = tentative_g + heuristic(neighbour, goal_cell);
          open_set.emplace(neighbour, f);
        }
      }
    }

    if (!found)
    {
      RCLCPP_WARN(logger_, "A*: no path found from (%d,%d) to (%d,%d).",
                  start_cell.x, start_cell.y, goal_cell.x, goal_cell.y);
      return result;
    }

    // Walk backwards from goal → start via came_from
    std::vector<CellIndex> cell_path;
    CellIndex current_cell = goal_cell;
    while (current_cell != start_cell)
    {
      cell_path.push_back(current_cell);
      current_cell = came_from[current_cell];
    }
    cell_path.push_back(start_cell);
    std::reverse(cell_path.begin(), cell_path.end());

    // Convert each cell back to a world-frame PoseStamped and append to path
    rclcpp::Time now = rclcpp::Clock().now();
    for (const CellIndex &cell : cell_path)
    {
      double wx, wy;
      gridToWorld(map, cell, wx, wy);

      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "map";
      pose.header.stamp = now;
      pose.pose.position.x = wx;
      pose.pose.position.y = wy;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.w = 1.0; // identity rotation; controller handles heading

      result.poses.push_back(pose);
    }

    RCLCPP_INFO(logger_, "A*: path found with %zu waypoints.", result.poses.size());
    return result;
  }

  // helpers
  bool PlannerCore::worldToGrid(
      const nav_msgs::msg::OccupancyGrid &map,
      double wx, double wy,
      CellIndex &out) const
  {
    const auto &info = map.info;

    // Translate relative to map origin
    double rel_x = wx - info.origin.position.x;
    double rel_y = wy - info.origin.position.y;

    int cx = static_cast<int>(std::floor(rel_x / info.resolution));
    int cy = static_cast<int>(std::floor(rel_y / info.resolution));

    if (cx < 0 || cy < 0 ||
        cx >= static_cast<int>(info.width) ||
        cy >= static_cast<int>(info.height))
    {
      return false;
    }

    out = CellIndex(cx, cy);
    return true;
  }

  void PlannerCore::gridToWorld(
      const nav_msgs::msg::OccupancyGrid &map,
      const CellIndex &idx,
      double &wx, double &wy) const
  {
    const auto &info = map.info;
    // Return the centre of the cell
    wx = info.origin.position.x + (idx.x + 0.5) * info.resolution;
    wy = info.origin.position.y + (idx.y + 0.5) * info.resolution;
  }

  bool PlannerCore::isTraversable(
      const nav_msgs::msg::OccupancyGrid &map,
      const CellIndex &idx) const
  {
    const auto &info = map.info;

    if (idx.x < 0 || idx.y < 0 ||
        idx.x >= static_cast<int>(info.width) ||
        idx.y >= static_cast<int>(info.height))
    {
      return false;
    }

    int8_t value = map.data[toMapIndex(map, idx)];

    // value == -1  → unknown (treat as obstacle)
    // value >= 50  → occupied
    // value <  50  → free
    return (value >= 0 && value < 50);
  }

  int PlannerCore::toMapIndex(
      const nav_msgs::msg::OccupancyGrid &map,
      const CellIndex &idx) const
  {
    return idx.y * static_cast<int>(map.info.width) + idx.x;
  }

  double PlannerCore::heuristic(const CellIndex &a, const CellIndex &b) const
  {
    // Euclidean distance (in cell units) — admissible for 8-connected grids
    double dx = static_cast<double>(a.x - b.x);
    double dy = static_cast<double>(a.y - b.y);
    return std::sqrt(dx * dx + dy * dy);
  }

  std::vector<PlannerCore::CellIndex> PlannerCore::getNeighbours(const CellIndex &idx) const
  {
    // 8-connected grid: cardinal + diagonal directions
    return {
        {idx.x - 1, idx.y},     // W
        {idx.x + 1, idx.y},     // E
        {idx.x, idx.y - 1},     // S
        {idx.x, idx.y + 1},     // N
        {idx.x - 1, idx.y - 1}, // SW
        {idx.x + 1, idx.y - 1}, // SE
        {idx.x - 1, idx.y + 1}, // NW
        {idx.x + 1, idx.y + 1}, // NE
    };
  }
}
