#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <cmath>

namespace robot
{

  class MapMemoryCore
  {
  public:
    explicit MapMemoryCore(const rclcpp::Logger &logger);

    // ── Main integration entry point ─────────────────────────────────────────
    // TODO: Called by the node's updateMap() when the robot has moved 1.5 m.
    //       Takes the latest costmap and the robot's current pose, fuses the
    //       costmap into global_map_, and returns it for publishing.
    void integrateCostmap(
        const nav_msgs::msg::OccupancyGrid &costmap,
        const geometry_msgs::msg::Pose &robot_pose);

    const nav_msgs::msg::OccupancyGrid getGlobalMap() const { return global_map_; };

  private:
    rclcpp::Logger logger_;

    // TODO: Flag that tracks whether global_map_ has been initialised yet.
    //       Set to true the first time integrateCostmap() is called.
    nav_msgs::msg::OccupancyGrid global_map_;
    bool map_initialized_ = false;

    // ── Global map constants ─────────────────────────────────────────────────
    // TODO: Choose a map size large enough to cover the expected environment.
    //       At 0.05 m/cell, 2000 × 2000 cells = 100 m × 100 m.
    static constexpr int GLOBAL_WIDTH = 2000;  // cells
    static constexpr int GLOBAL_HEIGHT = 2000; // cells
    static constexpr double RESOLUTION = 0.05; // metres per cell

    // ── Private helpers ──────────────────────────────────────────────────────

    // TODO: One-time setup for global_map_ on the very first integration call.
    //       Sets width, height, resolution, and origin.
    //       Fills the data vector with -1 (unknown) for every cell.
    void initGlobalMap(nav_msgs::msg::OccupancyGrid &global_map);

    // TODO: Convert a ROS quaternion to a yaw angle (radians).
    //       Use: yaw = atan2( 2*(w*z + x*y), 1 - 2*(y^2 + z^2) )
    double quaternionToYaw(const geometry_msgs::msg::Quaternion &q);
  };

} // namespace robot

#endif