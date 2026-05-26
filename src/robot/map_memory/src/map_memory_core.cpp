#include "map_memory_core.hpp"

namespace robot
{

  // constructor
  MapMemoryCore::MapMemoryCore(const rclcpp::Logger &logger)
      : logger_(logger) {}

  // update
  void MapMemoryCore::updateCostmap(const nav_msgs::msg::OccupancyGrid &msg)
  {
    latest_costmap_ = msg;
    has_costmap_ = true;
  }

  void MapMemoryCore::updateOdometry(const nav_msgs::msg::Odometry &msg)
  {
    robot_x_ = msg.pose.pose.position.x;
    robot_y_ = msg.pose.pose.position.y;

    // extract yaw from quarternion
    const auto &q = msg.pose.pose.orientation;
    robot_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                            1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  // check if threshold reached and merge possible
  bool MapMemoryCore::tryMerge()
  {
    if (!has_costmap_)
      return false;

    if (has_last_)
    {
      double dx = robot_x_ - last_x_;
      double dy = robot_y_ - last_y_;
      double dist = std::sqrt(dx * dx + dy * dy);
      if (dist < distance_threshold_)
        return false;
    }

    mergeLatestCostmap();

    last_x_ = robot_x_;
    last_y_ = robot_y_;
    has_last_ = true;
    return true;
  }

  // integration
  void MapMemoryCore::mergeLatestCostmap()
  {
    const auto &cm = latest_costmap_;
    const double cm_res = cm.info.resolution;
    const int cm_w = cm.info.width;
    const int cm_h = cm.info.height;
    const double cm_ox = cm.info.origin.position.x;
    const double cm_oy = cm.info.origin.position.y;

    // store outside of loop
    const double cos_y = std::cos(robot_yaw_);
    const double sin_y = std::sin(robot_yaw_);

    for (int cy = 0; cy < cm_h; ++cy)
    {
      for (int cx = 0; cx < cm_w; ++cx)
      {
        int8_t value = cm.data[cy * cm_w + cx];
        if (value < 0)
          continue;

        // convert cell to centre position
        double local_x = cm_ox + (cx + 0.5) * cm_res;
        double local_y = cm_oy + (cy + 0.5) * cm_res;

        // convert from local to world position
        double world_x = robot_x_ + local_x * cos_y - local_y * sin_y;
        double world_y = robot_y_ + local_x * sin_y + local_y * cos_y;

        // convert from world to local grid
        int g_x = static_cast<int>((world_x - origin_x_) / resolution_);
        int g_y = static_cast<int>((world_y - origin_y_) / resolution_);

        // check bounds
        if (g_x < 0 || g_x >= width_ || g_y < 0 || g_y >= height_)
          continue;

        // merge (keep max)
        if (value > global_map_[g_y * width_ + g_x])
        {
          global_map_[g_y * width_ + g_x] = value;
        }
      }
    }
  }

  // return global map
  nav_msgs::msg::OccupancyGrid MapMemoryCore::getGlobalMap() const
  {
    nav_msgs::msg::OccupancyGrid msg;
    msg.info.resolution = resolution_;
    msg.info.width = width_;
    msg.info.height = height_;
    msg.info.origin.position.x = origin_x_;
    msg.info.origin.position.y = origin_y_;
    msg.info.origin.orientation.w = 1.0;
    msg.data.assign(global_map_.begin(), global_map_.end());
    return msg;
  }

  // void MapMemoryCore::integrateCostmap(
  //     const nav_msgs::msg::OccupancyGrid &costmap,
  //     const geometry_msgs::msg::Pose &robot_pose)
  // {
  //   // initialize global map on first call
  //   if (!map_initialized_)
  //   {
  //     initGlobalMap(global_map_);
  //     map_initialized_ = true;
  //   }

  //   // read x and y from robot_pose.position
  //   double robot_x = robot_pose.position.x;
  //   double robot_y = robot_pose.position.y;

  //   // TODO: Convert the quaternion in robot_pose.orientation to a yaw angle
  //   //       using the quaternionToYaw() helper below
  //   double yaw = quaternionToYaw(robot_pose.orientation);

  //   // ── Convenience aliases ──────────────────────────────────────────────────
  //   double res = costmap.info.resolution;
  //   int cW = static_cast<int>(costmap.info.width);
  //   int cH = static_cast<int>(costmap.info.height);
  //   int gW = static_cast<int>(global_map_.info.width);
  //   int gH = static_cast<int>(global_map_.info.height);

  //   // ── Per-cell transform loop ──────────────────────────────────────────────
  //   for (int row = 0; row < cH; ++row)
  //   {
  //     for (int col = 0; col < cW; ++col)
  //     {

  //       // TODO: Look up the cell value in the costmap data vector.
  //       //       Index formula: row * cW + col
  //       int8_t cell_val = costmap.data[row * cW + col];

  //       // TODO: Skip unknown cells (-1) so they don't overwrite known global data
  //       if (cell_val == -1)
  //         continue;

  //       // ── Step 1: local position of this cell's centre ───────────────────
  //       // TODO: The costmap's info.origin is the position of its lower-left
  //       //       corner in the *robot's local frame*. Add half a cell to get
  //       //       the centre:
  //       //         local_x = costmap.info.origin.position.x + (col + 0.5) * res
  //       //         local_y = costmap.info.origin.position.y + (row + 0.5) * res
  //       double local_x = costmap.info.origin.position.x + (col + 0.5) * res;
  //       double local_y = costmap.info.origin.position.y + (row + 0.5) * res;

  //       // ── Step 2: rotate local → world frame ────────────────────────────
  //       // TODO: Apply a 2-D rotation by yaw, then translate by robot position:
  //       //         world_x = robot_x + local_x * cos(yaw) - local_y * sin(yaw)
  //       //         world_y = robot_y + local_x * sin(yaw) + local_y * cos(yaw)
  //       double world_x = robot_x + local_x * std::cos(yaw) - local_y * std::sin(yaw);
  //       double world_y = robot_y + local_x * std::sin(yaw) + local_y * std::cos(yaw);

  //       // ── Step 3: convert world position → global map grid index ────────
  //       // TODO: The global map's info.origin is its lower-left corner in the
  //       //       world frame. Subtract it, then divide by resolution:
  //       //         gx = (int)((world_x - global_map_.info.origin.position.x) / res)
  //       //         gy = (int)((world_y - global_map_.info.origin.position.y) / res)
  //       int gx = static_cast<int>((world_x - global_map_.info.origin.position.x) / res);
  //       int gy = static_cast<int>((world_y - global_map_.info.origin.position.y) / res);

  //       // ── Step 4: bounds check, then write ──────────────────────────────
  //       // TODO: Only write if (gx, gy) is inside the global map grid.
  //       //       Index formula: gy * gW + gx
  //       if (gx >= 0 && gx < gW && gy >= 0 && gy < gH)
  //       {
  //         global_map_.data[gy * gW + gx] = cell_val;
  //       }
  //     }
  //   }
  // }

  // ── Private: one-time global map initialisation ───────────────────────────────

  // void MapMemoryCore::initGlobalMap(nav_msgs::msg::OccupancyGrid &global_map_)
  // {
  //   // TODO: Set the resolution to match the incoming costmaps
  //   global_map_.info.resolution = RESOLUTION;

  //   // TODO: Set the grid dimensions (in cells)
  //   global_map_.info.width = GLOBAL_WIDTH;
  //   global_map_.info.height = GLOBAL_HEIGHT;

  //   // TODO: Centre the global map on the world origin so the robot starts
  //   //       in the middle of the grid:
  //   //         origin.x = -(GLOBAL_WIDTH  * RESOLUTION / 2.0)
  //   //         origin.y = -(GLOBAL_HEIGHT * RESOLUTION / 2.0)
  //   global_map_.info.origin.position.x = -(GLOBAL_WIDTH * RESOLUTION / 2.0);
  //   global_map_.info.origin.position.y = -(GLOBAL_HEIGHT * RESOLUTION / 2.0);
  //   global_map_.info.origin.orientation.w = 1.0; // identity quaternion

  //   // TODO: Allocate the data vector and fill it with -1 (unknown)
  //   global_map_.data.assign(GLOBAL_WIDTH * GLOBAL_HEIGHT, -1);

  //   RCLCPP_INFO(logger_, "Global map initialised: %d x %d cells at %.2f m/cell",
  //               GLOBAL_WIDTH, GLOBAL_HEIGHT, RESOLUTION);
  // }

  // ── Private: quaternion → yaw ─────────────────────────────────────────────────

  // double MapMemoryCore::quaternionToYaw(const geometry_msgs::msg::Quaternion &q)
  // {
  //   // TODO: Apply the standard yaw-from-quaternion formula:
  //   //         yaw = atan2( 2*(w*z + x*y),  1 - 2*(y^2 + z^2) )
  //   return std::atan2(
  //       2.0 * (q.w * q.z + q.x * q.y),
  //       1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  // }

} // namespace robot