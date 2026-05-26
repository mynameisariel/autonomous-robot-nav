#include <memory>
#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger()))
{
  // initialize constructs and their parameters
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/lidar",
      10,
      std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));

  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  costmap_.updateFromScan(*scan);

  auto grid = costmap_.getOccupancyGrid();
  grid.header = scan->header; // reuse scan's frame + timestamp
  costmap_pub_->publish(grid);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}