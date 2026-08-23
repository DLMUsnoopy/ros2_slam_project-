#pragma once

#include <cstdint>

#include "p03_slam_core/pose2d.hpp"

namespace p03_slam_core
{

// Contains a timestamped planar wheel-odometry pose.
struct Odometry2D
{
  std::int64_t timestamp_nanoseconds{0};  ///< Time when the pose was measured.
  Pose2D odom_from_base;
};

}  // namespace p03_slam_core
