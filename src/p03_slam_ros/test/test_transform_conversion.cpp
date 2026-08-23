#include <gtest/gtest.h>

#include <cmath>

#include "p03_slam_ros/transform_conversion.hpp"

namespace p03_slam_ros
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

TEST(TransformConversion, ConvertsTranslationAndYaw)
{
  geometry_msgs::msg::TransformStamped message;
  message.transform.translation.x = 0.175;
  message.transform.translation.y = -0.2;
  message.transform.rotation.z = std::sin(0.25 * kPi);
  message.transform.rotation.w = std::cos(0.25 * kPi);

  const p03_slam_core::Pose2D pose = toCorePose2D(message);

  EXPECT_DOUBLE_EQ(pose.x, 0.175);
  EXPECT_DOUBLE_EQ(pose.y, -0.2);
  EXPECT_NEAR(pose.yaw, 0.5 * kPi, 1e-12);
}

}  // namespace
}  // namespace p03_slam_ros
