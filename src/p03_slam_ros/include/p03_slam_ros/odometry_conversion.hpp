#pragma once

#include "nav_msgs/msg/odometry.hpp"
#include "p03_slam_core/odometry2d.hpp"

namespace p03_slam_ros
{

// Converts a ROS 2 odometry message into the ROS-independent core type.
[[nodiscard]] p03_slam_core::Odometry2D toCoreOdometry(
  const nav_msgs::msg::Odometry & message);

}  // namespace p03_slam_ros
