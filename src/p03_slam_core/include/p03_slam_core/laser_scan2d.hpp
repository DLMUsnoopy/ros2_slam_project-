#pragma once

#include <cstdint>
#include <vector>

namespace p03_slam_core
{

/// Contains one planar laser scan without ROS-specific message types.
struct LaserScan2D
{
  std::int64_t timestamp_nanoseconds{0};  ///< Capture time of the first range measurement.
  double angle_min{0.0};                 ///< Angle of the first beam, in radians.
  double angle_increment{0.0};           ///< Angular distance between beams, in radians.
  double time_increment{0.0};            ///< Time between beams, in seconds.
  double range_min{0.0};                 ///< Minimum valid range, in meters.
  double range_max{0.0};                 ///< Maximum valid range, in meters.
  std::vector<float> ranges;              ///< Range measurements, in meters.
};

}  // namespace p03_slam_core
