#pragma once

#include "p03_slam_core/laser_scan2d.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace p03_slam_ros
{

// Converts a ROS 2 laser scan message into the ROS-independent core type.
[[nodiscard]] p03_slam_core::LaserScan2D toCoreLaserScan(
  const sensor_msgs::msg::LaserScan & message);

}  // namespace p03_slam_ros
