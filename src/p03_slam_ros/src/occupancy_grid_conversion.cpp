#include "p03_slam_ros/occupancy_grid_conversion.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace p03_slam_ros
{

// Converts map metadata and log-odds cells into the ROS occupancy representation.
nav_msgs::msg::OccupancyGrid toRosOccupancyGrid(
  const p03_slam_core::OccupancyGrid2D & grid)
{
  nav_msgs::msg::OccupancyGrid message;
  const p03_slam_core::OccupancyGridConfig & config = grid.config();

  message.info.resolution = static_cast<float>(config.resolution);
  message.info.width = static_cast<std::uint32_t>(config.width);
  message.info.height = static_cast<std::uint32_t>(config.height);
  message.info.origin.position.x = config.origin.x;
  message.info.origin.position.y = config.origin.y;
  message.info.origin.orientation.w = 1.0;
  message.data.reserve(grid.cellCount());

  constexpr std::int8_t kUnknownOccupancy = -1;
  for (std::size_t y = 0; y < config.height; ++y) {
    for (std::size_t x = 0; x < config.width; ++x) {
      const p03_slam_core::GridIndex index{
        static_cast<int>(x), static_cast<int>(y)};

      if (!grid.isObserved(index)) {
        message.data.push_back(kUnknownOccupancy);
        continue;
      }

      const double log_odds = static_cast<double>(grid.logOdds(index));
      const double probability = 1.0 / (1.0 + std::exp(-log_odds));
      message.data.push_back(static_cast<std::int8_t>(
        std::lround(100.0 * probability)));
    }
  }

  return message;
}

}  // namespace p03_slam_ros
