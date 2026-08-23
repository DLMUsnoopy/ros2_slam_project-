#include <gtest/gtest.h>

#include <cmath>

#include "p03_slam_ros/odometry_conversion.hpp"

namespace p03_slam_ros
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

TEST(OdometryConversion, CopiesTimestampAndConvertsPlanarPose)
{
  nav_msgs::msg::Odometry message;
  message.header.stamp.sec = 8;
  message.header.stamp.nanosec = 250000000U;
  message.pose.pose.position.x = 1.5;
  message.pose.pose.position.y = -2.0;
  message.pose.pose.orientation.z = std::sin(0.25 * kPi);
  message.pose.pose.orientation.w = std::cos(0.25 * kPi);

  const p03_slam_core::Odometry2D odometry = toCoreOdometry(message);

  EXPECT_EQ(odometry.timestamp_nanoseconds, 8250000000LL);
  EXPECT_DOUBLE_EQ(odometry.odom_from_base.x, 1.5);
  EXPECT_DOUBLE_EQ(odometry.odom_from_base.y, -2.0);
  EXPECT_NEAR(odometry.odom_from_base.yaw, 0.5 * kPi, 1e-12);
}

}  // namespace
}  // namespace p03_slam_ros
