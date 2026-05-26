#include "planner_node.hpp"

// constructor
PlannerNode::PlannerNode() : Node("planner"), state_(State::WAITING_FOR_GOAL), planner_(robot::PlannerCore(this->get_logger()))
{
  // subscribers
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  // publisher
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  // timer
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));

  RCLCPP_INFO(this->get_logger(), "PlannerNode initialised. Waiting for goal...");
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  current_map_ = *msg;
  map_received_ = true;
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL)
  {
    RCLCPP_INFO(this->get_logger(), "Map updated - replanning.");
    planPath();
  }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  goal_ = *msg;
  goal_received_ = true;

  RCLCPP_INFO(this->get_logger(), "New goal received: (%.2f, %.2f).",
              goal_.point.x, goal_.point.y);

  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  planPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  robot_pose_ = msg->pose.pose;
}

void PlannerNode::timerCallback()
{
  if (state_ != State::WAITING_FOR_ROBOT_TO_REACH_GOAL)
  {
    return;
  }

  if (goalReached())
  {
    RCLCPP_INFO(this->get_logger(), "Goal reached!");
    state_ = State::WAITING_FOR_GOAL;
  }
  else
  {
    // Periodic replan: accounts for dynamic obstacles and odometry drift
    RCLCPP_DEBUG(this->get_logger(), "Timer replan.");
    planPath();
  }
}

bool PlannerNode::goalReached() const
{
  double dx = goal_.point.x - robot_pose_.position.x;
  double dy = goal_.point.y - robot_pose_.position.y;
  return std::sqrt(dx * dx + dy * dy) < 0.5;
}

void PlannerNode::planPath()
{
  if (!goal_received_ || current_map_.data.empty())
  {
    RCLCPP_WARN(this->get_logger(), "Cannot plan path: missing map or goal!");
    return;
  }

  if (!map_received_ || current_map_.data.empty())
  {
    RCLCPP_WARN(this->get_logger(), "Cannot plan: map not yet available.");
    return;
  }

  // A* implementation
  nav_msgs::msg::Path path = planner_.computePath(current_map_, robot_pose_, goal_);

  if (path.poses.empty())
  {
    RCLCPP_WARN(this->get_logger(), "Planner returned an empty path.");
    return;
  }

  // stamp and publish
  path.header.stamp = this->get_clock()->now();
  path.header.frame_id = "map";
  path_pub_->publish(path);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
