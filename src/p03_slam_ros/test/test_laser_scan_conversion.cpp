#include <gtest/gtest.h>

#include "p03_slam_ros/laser_scan_conversion.hpp"

namespace p03_slam_ros
{
namespace
{

TEST(LaserScanConversion, CopiesFieldsUsedByCore)
{
  sensor_msgs::msg::LaserScan message;
  message.header.stamp.sec = 12;
  message.header.stamp.nanosec = 345000000U;
  message.angle_min = -1.0F;
  message.angle_increment = 0.1F;
  message.time_increment = 0.002F;
  message.range_min = 0.1F;
  message.range_max = 50.0F;
  message.ranges = {1.0F, 2.5F, 50.0F};

  const p03_slam_core::LaserScan2D scan = toCoreLaserScan(message);

  EXPECT_EQ(scan.timestamp_nanoseconds, 12345000000LL);
  EXPECT_DOUBLE_EQ(scan.angle_min, static_cast<double>(message.angle_min));
  EXPECT_DOUBLE_EQ(scan.angle_increment, static_cast<double>(message.angle_increment));
  EXPECT_DOUBLE_EQ(scan.time_increment, static_cast<double>(message.time_increment));
  EXPECT_DOUBLE_EQ(scan.range_min, static_cast<double>(message.range_min));
  EXPECT_DOUBLE_EQ(scan.range_max, static_cast<double>(message.range_max));
  EXPECT_EQ(scan.ranges, message.ranges);
}

}  // namespace
}  // namespace p03_slam_ros
