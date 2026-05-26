#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include <vector>
#include <unordered_map>
#include <queue>

namespace robot
{

  class PlannerCore
  {
  public:
    explicit PlannerCore(const rclcpp::Logger &logger);

    // A* algorithm on the occupancy grid - returns nav_msgs::msg::Path
    nav_msgs::msg::Path computePath(const nav_msgs::msg::OccupancyGrid &map,
                                    const geometry_msgs::msg::Pose &start,
                                    const geometry_msgs::msg::PointStamped &goal);

  private:
    rclcpp::Logger logger_;

    // 2D grid index
    struct CellIndex
    {
      int x;
      int y;

      CellIndex(int xx, int yy) : x(xx), y(yy) {}
      CellIndex() : x(0), y(0) {}

      bool operator==(const CellIndex &other) const
      {
        return (x == other.x && y == other.y);
      }

      bool operator!=(const CellIndex &other) const
      {
        return (x != other.x || y != other.y);
      }
    };

    // Hash function for CellIndex so it can be used in std::unordered_map
    struct CellIndexHash
    {
      std::size_t operator()(const CellIndex &idx) const
      {
        // A simple hash combining x and y
        return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
      }
    };

    // Structure representing a node in the A* open set
    struct AStarNode
    {
      CellIndex index;
      double f_score; // f = g + h

      AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}

      // Min-heap: smallest f on top
      bool operator>(const AStarNode &o) const { return f_score > o.f_score; }
    };

    // Comparator for the priority queue (min-heap by f_score)
    struct CompareF
    {
      bool operator()(const AStarNode &a, const AStarNode &b)
      {
        // We want the node with the smallest f_score on top
        return a.f_score > b.f_score;
      }
    };

    // coordinate conversion

    /** World (metres) → grid cell. Returns false if the point is outside the grid. */
    bool worldToGrid(const nav_msgs::msg::OccupancyGrid &map,
                     double wx, double wy,
                     CellIndex &out) const;

    /** Grid cell centre → world (metres). */
    void gridToWorld(const nav_msgs::msg::OccupancyGrid &map,
                     const CellIndex &idx,
                     double &wx, double &wy) const;

    /** True if the cell is within bounds and not an obstacle (value < 50). */
    bool isTraversable(const nav_msgs::msg::OccupancyGrid &map,
                       const CellIndex &idx) const;

    /** Flat row-major index into map.data. */
    int toMapIndex(const nav_msgs::msg::OccupancyGrid &map,
                   const CellIndex &idx) const;

    /** Euclidean heuristic in cell units. */
    double heuristic(const CellIndex &a, const CellIndex &b) const;

    /** Return the up-to-8 neighbours of a cell (4-connected or 8-connected). */
    std::vector<CellIndex> getNeighbours(const CellIndex &idx) const;
  };

}

#endif
