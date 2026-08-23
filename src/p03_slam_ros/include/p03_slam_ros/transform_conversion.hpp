#pragma once

#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "p03_slam_core/pose2d.hpp"

namespace p03_slam_ros
{

// Converts a ROS 2 transform into a planar core pose.
[[nodiscard]] p03_slam_core::Pose2D toCorePose2D(
  const geometry_msgs::msg::TransformStamped & message);

// Converts a planar core pose into a ROS 2 transform.
[[nodiscard]] geometry_msgs::msg::TransformStamped toRosTransformStamped(
  const p03_slam_core::Pose2D & pose,
  const std::string & parent_frame,
  const std::string & child_frame);

}  // namespace p03_slam_ros
