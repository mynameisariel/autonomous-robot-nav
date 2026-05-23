#include "costmap_core.hpp"

namespace robot
{

// constructor - initialize grid to all 0s
CostmapCore::CostmapCore(const rclcpp::Logger& logger)
    : logger_(logger), grid_(width_ * height_, 0) {}

// initializeCostmap - reset all values to 0
void CostmapCore::initializeCostmap()
{
  grid_.assign(width_ * height_, 0);
}

// convertToGrid - polar (sensor frame) to flat grid index
bool CostmapCore::convertToGrid(double range, double angle, int& x_grid, int& y_grid) const
{
    // 1: polar to Cartesian
    double x_m = range * std::cos(angle);
    double y_m = range * std::sin(angle);

    // 2: shift so that the robot (0,0) maps to grid centre
    x_grid = static_cast<int>((x_m - origin_x_) / resolution_);
    y_grid = static_cast<int>((y_m - origin_y_) / resolution_);

    return (x_grid >= 0 && x_grid < width_ && y_grid >= 0 && y_grid < height_);
}

// markObstacle - set a single cell to cost 100 ("definitely occupied")
void CostmapCore::markObstacle(int x_grid, int y_grid)
{
    // check bounds
    grid_[y_grid * width_ + x_grid] = max_cost_;
}

// updateFromScan - laserscan update for each lidar scan
void CostmapCore::updateFromScan(const sensor_msgs::msg::LaserScan& scan) {
  initializeCostmap();

  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    double range = scan.ranges[i];
    double angle = scan.angle_min + i * scan.angle_increment;

    if (range < scan.range_min || range > scan.range_max || std::isnan(range)) {
      continue;
    }

    int x_grid, y_grid;
    if (convertToGrid(range, angle, x_grid, y_grid)) {
      markObstacle(x_grid, y_grid);
    }
  }

  inflateObstacles();
}


// inflateObstacles - spread cost outward from every obstacle cell
void CostmapCore::inflateObstacles()
{
    // How many cells does the inflation radius span?
    int inflate_cells = static_cast<int>(inflation_radius_ / resolution_);

    // take a snapshot of the current map so that inflation from one obstacle does not affect the distance calculation for a neighbouring obstacle
    std::vector<std::pair<int, int>> obstacles;
    for (int y = 0; y < height_; ++y) {
      for (int x = 0; x < width_; ++x) {
        if (grid_[y * width_ + x] == max_cost_) {
          obstacles.emplace_back(x, y);
        }
      }
    }

    // create buffer around each obstacle
    for (const auto& [ox, oy] : obstacles) {
      for (int dy = -inflate_cells; dy <= inflate_cells; ++dy) {
        for (int dx = -inflate_cells; dx <= inflate_cells; ++dx) {
          int nx = ox + dx;
          int ny = oy + dy;

          // skip out-of-bounds
          if (nx < 0 || nx >= width_ || ny < 0 || ny >= height_) continue;

          double dist_cells = std::sqrt(dx * dx + dy * dy);
          double dist_meters = dist_cells * resolution_;
          if (dist_meters > inflation_radius_) continue;

          int new_cost = static_cast<int>(max_cost_ * (1.0 - dist_meters / inflation_radius_));

          // only write if new cost beats existing cost
          int idx = ny * width_ + nx;
          if (new_cost > grid_[idx]) {
            grid_[idx] = new_cost;
          }
        }
      }
    }
}

// publish costmap - pack costmap_data_ into an OccupancyGrid and publish
nav_msgs::msg::OccupancyGrid CostmapCore::getOccupancyGrid() const {
  nav_msgs::msg::OccupancyGrid msg;
  msg.info.resolution = resolution_;
  msg.info.width = width_;
  msg.info.height = height_;
  msg.info.origin.position.x = origin_x_;
  msg.info.origin.position.y = origin_y_;
  msg.info.origin.orientation.w = 1.0;
  msg.data.assign(grid_.begin(), grid_.end()); 
  return msg;
}

}  // namespace robot