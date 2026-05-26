#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <cmath>

namespace robot
{

  class MapMemoryCore
  {
  public:
    explicit MapMemoryCore(const rclcpp::Logger &logger);

    void updateCostmap(const nav_msgs::msg::OccupancyGrid &msg);
    void updateOdometry(const nav_msgs::msg::Odometry &msg);

    // called from timer (updateMap) - checks if robot has moved grreater than threshold distance since last merge
    bool tryMerge();

    // main integration entry point
    // TODO: Called by the node's updateMap() when the robot has moved 1.5 m.
    //       Takes the latest costmap and the robot's current pose, fuses the
    //       costmap into global_map_, and returns it for publishing.
    // void integrateCostmap(
    //     const nav_msgs::msg::OccupancyGrid &costmap,
    //     const geometry_msgs::msg::Pose &robot_pose);

    // return copy of global map as OccupancyGrid message - called every timer tick
    nav_msgs::msg::OccupancyGrid getGlobalMap() const;

  private:
    rclcpp::Logger logger_;

    // global map storage
    std::vector<int8_t> global_map_; // flat vector of int8_t of size width*height, intialized to -1

    // global math constants
    const int width_ = 2000;
    const int height_ = 2000;
    const double resolution_ = 0.05;
    const double origin_x_ = -(2000 * 0.05 / 2.0); // -50.0 m
    const double origin_y_ = -(2000 * 0.05 / 2.0); // -50.0 m

    // latest costmap (updateCostmpa)
    nav_msgs::msg::OccupancyGrid latest_costmap_;
    bool has_costmap_ = false;

    // robot pose (updateOdometry)
    double robot_x_ = 0.0;
    double robot_y_ = 0.0;
    double robot_yaw_ = 0.0;

    // distance tracing for robot position at time of most recent merge
    double last_x_ = 0.0;
    double last_y_ = 0.0;
    bool has_last_ = false;

    const double distance_threshold_ = 1.5; // metres

    // Transforms every known cell in latest_costmap_ into the global frame and writes it into global_map_
    void mergeLatestCostmap();
  };

}

#endif