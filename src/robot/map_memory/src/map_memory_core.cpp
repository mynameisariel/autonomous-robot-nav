#include "map_memory_core.hpp"

namespace robot
{

  // ── Constructor ───────────────────────────────────────────────────────────────

  MapMemoryCore::MapMemoryCore(const rclcpp::Logger &logger)
      : logger_(logger) {}

  // ── Public: main integration entry point ─────────────────────────────────────

  void MapMemoryCore::integrateCostmap(
      const nav_msgs::msg::OccupancyGrid &costmap,
      const geometry_msgs::msg::Pose &robot_pose)
  {
    // TODO: On the very first call, set up the global map's metadata and fill
    //       every cell with -1 (unknown). Guard with map_initialized_.
    if (!map_initialized_)
    {
      initGlobalMap(global_map_);
      map_initialized_ = true;
    }

    // ── Extract robot pose ───────────────────────────────────────────────────
    // TODO: Read x and y from robot_pose.position
    double robot_x = robot_pose.position.x;
    double robot_y = robot_pose.position.y;

    // TODO: Convert the quaternion in robot_pose.orientation to a yaw angle
    //       using the quaternionToYaw() helper below
    double yaw = quaternionToYaw(robot_pose.orientation);

    // ── Convenience aliases ──────────────────────────────────────────────────
    double res = costmap.info.resolution;
    int cW = static_cast<int>(costmap.info.width);
    int cH = static_cast<int>(costmap.info.height);
    int gW = static_cast<int>(global_map_.info.width);
    int gH = static_cast<int>(global_map_.info.height);

    // ── Per-cell transform loop ──────────────────────────────────────────────
    for (int row = 0; row < cH; ++row)
    {
      for (int col = 0; col < cW; ++col)
      {

        // TODO: Look up the cell value in the costmap data vector.
        //       Index formula: row * cW + col
        int8_t cell_val = costmap.data[row * cW + col];

        // TODO: Skip unknown cells (-1) so they don't overwrite known global data
        if (cell_val == -1)
          continue;

        // ── Step 1: local position of this cell's centre ───────────────────
        // TODO: The costmap's info.origin is the position of its lower-left
        //       corner in the *robot's local frame*. Add half a cell to get
        //       the centre:
        //         local_x = costmap.info.origin.position.x + (col + 0.5) * res
        //         local_y = costmap.info.origin.position.y + (row + 0.5) * res
        double local_x = costmap.info.origin.position.x + (col + 0.5) * res;
        double local_y = costmap.info.origin.position.y + (row + 0.5) * res;

        // ── Step 2: rotate local → world frame ────────────────────────────
        // TODO: Apply a 2-D rotation by yaw, then translate by robot position:
        //         world_x = robot_x + local_x * cos(yaw) - local_y * sin(yaw)
        //         world_y = robot_y + local_x * sin(yaw) + local_y * cos(yaw)
        double world_x = robot_x + local_x * std::cos(yaw) - local_y * std::sin(yaw);
        double world_y = robot_y + local_x * std::sin(yaw) + local_y * std::cos(yaw);

        // ── Step 3: convert world position → global map grid index ────────
        // TODO: The global map's info.origin is its lower-left corner in the
        //       world frame. Subtract it, then divide by resolution:
        //         gx = (int)((world_x - global_map_.info.origin.position.x) / res)
        //         gy = (int)((world_y - global_map_.info.origin.position.y) / res)
        int gx = static_cast<int>((world_x - global_map_.info.origin.position.x) / res);
        int gy = static_cast<int>((world_y - global_map_.info.origin.position.y) / res);

        // ── Step 4: bounds check, then write ──────────────────────────────
        // TODO: Only write if (gx, gy) is inside the global map grid.
        //       Index formula: gy * gW + gx
        if (gx >= 0 && gx < gW && gy >= 0 && gy < gH)
        {
          global_map_.data[gy * gW + gx] = cell_val;
        }
      }
    }
  }

  // ── Private: one-time global map initialisation ───────────────────────────────

  void MapMemoryCore::initGlobalMap(nav_msgs::msg::OccupancyGrid &global_map_)
  {
    // TODO: Set the resolution to match the incoming costmaps
    global_map_.info.resolution = RESOLUTION;

    // TODO: Set the grid dimensions (in cells)
    global_map_.info.width = GLOBAL_WIDTH;
    global_map_.info.height = GLOBAL_HEIGHT;

    // TODO: Centre the global map on the world origin so the robot starts
    //       in the middle of the grid:
    //         origin.x = -(GLOBAL_WIDTH  * RESOLUTION / 2.0)
    //         origin.y = -(GLOBAL_HEIGHT * RESOLUTION / 2.0)
    global_map_.info.origin.position.x = -(GLOBAL_WIDTH * RESOLUTION / 2.0);
    global_map_.info.origin.position.y = -(GLOBAL_HEIGHT * RESOLUTION / 2.0);
    global_map_.info.origin.orientation.w = 1.0; // identity quaternion

    // TODO: Allocate the data vector and fill it with -1 (unknown)
    global_map_.data.assign(GLOBAL_WIDTH * GLOBAL_HEIGHT, -1);

    RCLCPP_INFO(logger_, "Global map initialised: %d x %d cells at %.2f m/cell",
                GLOBAL_WIDTH, GLOBAL_HEIGHT, RESOLUTION);
  }

  // ── Private: quaternion → yaw ─────────────────────────────────────────────────

  double MapMemoryCore::quaternionToYaw(const geometry_msgs::msg::Quaternion &q)
  {
    // TODO: Apply the standard yaw-from-quaternion formula:
    //         yaw = atan2( 2*(w*z + x*y),  1 - 2*(y^2 + z^2) )
    return std::atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

} // namespace robot