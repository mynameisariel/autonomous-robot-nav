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

// timer callback
void MapMemoryNode::updateMap()
{
  map_memory_.tryMerge(); // try but don't care ab return value

  auto grid = map_memory_.getGlobalMap();
  grid.header.stamp = this->get_clock()->now();
  grid.header.frame_id = "robot"; // global frame
  map_pub_->publish(grid);        // always publish
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}