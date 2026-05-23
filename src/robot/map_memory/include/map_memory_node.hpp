#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node
{
public:
  MapMemoryNode();

  // Subscriber callbacks
  void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom);

private:
  robot::MapMemoryCore map_memory_;

  // TODO: Rename these to match what they actually are:
  //   - costmap_sub_  subscribes to "/costmap"       (OccupancyGrid)
  //   - odom_sub_     subscribes to "/odom/filtered" (Odometry)
  //   - map_pub_      publishes   to "/map"           (OccupancyGrid)
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;

  // TODO: Declare the 1 Hz wall timer
  rclcpp::TimerBase::SharedPtr timer_;

  // TODO: Declare the timer callback
  void updateMap();

  // ── Robot state ──────────────────────────────────────────────────────────
  // TODO: Store the most recent costmap received from /costmap
  nav_msgs::msg::OccupancyGrid latest_costmap_;

  // TODO: Store the robot's most recent pose (updated every odom message)
  geometry_msgs::msg::Pose current_pose_;

  // TODO: Record the robot's position at the time of the last map update
  //       so we can measure how far it has moved since then
  double last_update_x_;
  double last_update_y_;

  // TODO: Declare a constant for the minimum travel distance (1.5 m)
  //       before a new map integration is triggered
  const double distance_threshold_ = 1.5;

  // ── Control flags ────────────────────────────────────────────────────────
  // TODO: Set to true in costmapCallback when a new costmap arrives
  bool costmap_updated_ = false;

  // TODO: Set to true in odomCallback when the robot has moved >= 1.5 m
  //       Reset to false in updateMap() after each integration
  bool should_update_map_ = false;
};

#endif