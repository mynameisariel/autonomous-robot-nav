#include "map_memory_node.hpp"

// constructor
MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger()))
{
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/costmap", 10,
      std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10,
      std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
  timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&MapMemoryNode::updateMap, this));
}

// CALLBACKS
// costmap callback for costmap updates
void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  map_memory_.updateCostmap(*msg);
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  map_memory_.updateOdometry(*msg);
}

// void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
// {
//   // Save the robot's current pose so integrateCostmap() has it
//   current_pose_ = odom->pose.pose;

//   // Extract the current x and y position
//   double x = current_pose_.position.x;
//   double y = current_pose_.position.y;

//   // compute distance
//   double distance = std::sqrt(
//       std::pow(x - last_update_x_, 2) +
//       std::pow(y - last_update_y_, 2));

//   // record the new reference position and flag that a map update is due
//   if (distance >= distance_threshold_)
//   {
//     last_update_x_ = x;
//     last_update_y_ = y;
//     should_update_map_ = true;
//   }
// }

// timer callback
void MapMemoryNode::updateMap()
{
  map_memory_.tryMerge(); // try but don't care ab return value

  auto grid = map_memory_.getGlobalMap();
  grid.header.stamp = this->get_clock()->now();
  grid.header.frame_id = "robot"; // global frame
  map_pub_->publish(grid);        // always publish
}

// void MapMemoryNode::updateMap()
// {
//   // TODO: Only integrate and publish when:
//   //         (a) the robot has moved far enough (should_update_map_), AND
//   //         (b) at least one costmap has arrived (costmap_updated_)
//   if (!should_update_map_ || !costmap_updated_)
//   {
//     return;
//   }

//   // TODO: Delegate the actual grid fusion to MapMemoryCore
//   //       Pass: latest_costmap_, current_pose_, and a reference to global_map_
//   //       (You will need to add global_map_ as a member of this node or of the core)
//   map_memory_.integrateCostmap(latest_costmap_, current_pose_);

//   // publish the updated global map
//   map_pub_->publish(map_memory_.getGlobalMap());

//   // reset the flag so we don't publish again until the robot moves another 1.5 m
//   should_update_map_ = false;
// }

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}