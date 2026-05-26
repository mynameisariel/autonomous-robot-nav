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

private:
  robot::MapMemoryCore map_memory_;

  // subscribers and publishers
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;

  // wall timer
  rclcpp::TimerBase::SharedPtr timer_;

  void updateMap();

  // Subscriber callbacks
  void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  // store the most recent callback
  // nav_msgs::msg::OccupancyGrid latest_costmap_;

  // // store robot's most recent post
  // geometry_msgs::msg::Pose current_pose_;

  // // record the robot's position at the time of the last map update so we can measure how far it has moved since then
  // double last_update_x_;
  // double last_update_y_;

  // // declare a constant for the minimum travel distance (1.5 m) before a new map integration is triggered
  // const double distance_threshold_ = 1.5;

  // // set to true in costmapCallback when a new costmap arrives
  // bool costmap_updated_ = false;

  // // set to true in odomCallback when the robot has moved >= 1.5 m, then reset to false in updateMap() after each integration
  // bool should_update_map_ = false;
};

#endif