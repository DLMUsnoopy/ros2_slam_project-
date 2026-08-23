#include "p03_slam_ros/laser_scan_conversion.hpp"

#include <cstdint>

namespace p03_slam_ros
{

// Converts a ROS 2 message into the data used by the core SLAM algorithms.
p03_slam_core::LaserScan2D toCoreLaserScan(const sensor_msgs::msg::LaserScan & message)
{
  constexpr std::int64_t kNanosecondsPerSecond = 1000000000LL;

  p03_slam_core::LaserScan2D scan;
  scan.timestamp_nanoseconds =
    static_cast<std::int64_t>(message.header.stamp.sec) * kNanosecondsPerSecond +
    static_cast<std::int64_t>(message.header.stamp.nanosec);   ///original timestamp : seconds + nanoseconds -> only nanoseconds reporesentation 
  scan.angle_min = message.angle_min;
  scan.angle_increment = message.angle_increment;
  scan.time_increment = message.time_increment;
  scan.range_min = message.range_min;
  scan.range_max = message.range_max;
  scan.ranges = message.ranges;
  return scan;
}

}  // namespace p03_slam_ros
