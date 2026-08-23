#pragma once

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "p03_slam_core/occupancy_grid2d.hpp"

namespace p03_slam_ros
{

// Converts a core occupancy grid into a ROS 2 map message.
[[nodiscard]] nav_msgs::msg::OccupancyGrid toRosOccupancyGrid(
  const p03_slam_core::OccupancyGrid2D & grid);

}  // namespace p03_slam_ros
