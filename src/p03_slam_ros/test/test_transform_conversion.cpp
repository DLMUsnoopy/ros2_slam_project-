#include <gtest/gtest.h>

#include <cmath>

#include "p03_slam_ros/transform_conversion.hpp"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

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

TEST(TransformConversion, ConvertsPoseToStampedTransform)
{
  const geometry_msgs::msg::TransformStamped message = toRosTransformStamped(
    {1.5, -2.0, 0.5 * kPi}, "map", "odom");

  EXPECT_EQ(message.header.frame_id, "map");
  EXPECT_EQ(message.child_frame_id, "odom");
  EXPECT_DOUBLE_EQ(message.transform.translation.x, 1.5);
  EXPECT_DOUBLE_EQ(message.transform.translation.y, -2.0);
  EXPECT_NEAR(toCorePose2D(message).yaw, 0.5 * kPi, 1e-12);
}

TEST(TransformConversion, ConvertsPoseToStampedPose)
{
  const p03_slam_core::Pose2D pose{1.0, -2.0, 0.5};

  const geometry_msgs::msg::PoseStamped message =
    p03_slam_ros::toRosPoseStamped(pose, "map");

  EXPECT_EQ(message.header.frame_id, "map");
  EXPECT_DOUBLE_EQ(message.pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(message.pose.position.y, -2.0);
  EXPECT_NEAR(tf2::getYaw(message.pose.orientation), 0.5, 1e-12);
}

}  // namespace
}  // namespace p03_slam_ros
