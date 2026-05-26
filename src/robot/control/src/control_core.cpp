#include "control_core.hpp"

#include <cmath>

namespace robot
{

  ControlCore::ControlCore(const rclcpp::Logger &logger)
      : logger_(logger) {}

  void ControlCore::updatePath(const nav_msgs::msg::Path &msg)
  {
    path_ = msg;
    has_path_ = true;
  }

  // compute robot position
  void ControlCore::updateOdometry(const nav_msgs::msg::Odometry &msg)
  {
    robot_x_ = msg.pose.pose.position.x;
    robot_y_ = msg.pose.pose.position.y;

    // extract yaw from quarternion
    const auto &q = msg.pose.pose.orientation;
    robot_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                            1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  geometry_msgs::msg::Twist ControlCore::computeCommand()
  {
    // set to default zero
    geometry_msgs::msg::Twist twist;

    if (!has_path_ || path_.poses.empty())
    {
      return twist;
    }

    // stop if reached near end of path
    const auto &last = path_.poses.back();
    double dx_end = last.pose.position.x - robot_x_;
    double dy_end = last.pose.position.y - robot_y_;
    double dist_to_end = std::sqrt(dx_end * dx_end + dy_end * dy_end);
    if (dist_to_end < goal_tolerance_)
    {
      return twist;
    }

    // find lookahead point
    auto target = extractLookAheadPoint();
    if (!target.has_value())
    {
      return twist;
    }

    // angle from robot's position to lookahead point (world frame)
    double dx = target->pose.position.x - robot_x_;
    double dy = target->pose.position.y - robot_y_;
    double angle_to_target = std::atan2(dy, dx);

    // calculate difference between target angle and our heading
    double head_difference = angle_to_target - robot_yaw_;

    // wrap to [-pi, pi] so we always turn the short way
    while (head_difference > M_PI)
      head_difference -= 2.0 * M_PI;
    while (head_difference < -M_PI)
      head_difference += 2.0 * M_PI;

    // cap steering angle
    if (std::abs(head_difference) > max_steering_angle_)
    {
      twist.linear.x = 0; // stop and rotate in place
    }
    else
    {
      twist.linear.x = linear_speed_;
    }
    head_difference = std::max(-max_steering_angle_, std::min(head_difference, max_steering_angle_));
    twist.angular.z = 2.0 * head_difference;

    return twist;
  }

  std::optional<geometry_msgs::msg::PoseStamped> ControlCore::extractLookAheadPoint() const
  {
    if (!has_path_ || path_.poses.empty())
      return std::nullopt;

    for (const auto &pose : path_.poses)
    {
      double dx = pose.pose.position.x - robot_x_;
      double dy = pose.pose.position.y - robot_y_;
      double d = std::sqrt(dx * dx + dy * dy);

      // forward only
      double to_pose = std::atan2(dy, dx);
      double angle_difference = to_pose - robot_yaw_;

      while (angle_difference > M_PI)
      {
        angle_difference -= 2 * M_PI;
      }

      while (angle_difference < -M_PI)
      {
        angle_difference += 2 * M_PI;
      }

      if (std::abs(angle_difference) > M_PI_2)
      {
        continue;
      }

      if (d >= ahead_distance_)
      {
        return pose;
      }
    }

    // all waypoints are within lookahead -> we're near the end; aim at last one
    return path_.poses.back();
  }

  bool ControlCore::hasPath() const
  {
    return has_path_ && !path_.poses.empty();
  }

}