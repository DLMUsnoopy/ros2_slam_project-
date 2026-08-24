#include <cmath>
#include <cstdint>

#include "gtest/gtest.h"
#include "p03_slam_core/odometry_buffer2d.hpp"

namespace p03_slam_core
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

TEST(OdometryBuffer2DTest, InterpolatesPositionAtSensorTimestamp)
{
  OdometryBuffer2D buffer;
  buffer.add({1000000000LL, {1.0, 2.0, 0.2}});
  buffer.add({1100000000LL, {3.0, 4.0, 0.6}});

  const std::optional<Pose2D> pose = buffer.poseAt(1050000000LL);

  ASSERT_TRUE(pose.has_value());
  EXPECT_DOUBLE_EQ(pose->x, 2.0);
  EXPECT_DOUBLE_EQ(pose->y, 3.0);
  EXPECT_NEAR(pose->yaw, 0.4, 1e-12);
}

TEST(OdometryBuffer2DTest, InterpolatesYawAcrossAngleBoundary)
{
  OdometryBuffer2D buffer;
  buffer.add({100LL, {0.0, 0.0, 170.0 * kPi / 180.0}});
  buffer.add({200LL, {0.0, 0.0, -170.0 * kPi / 180.0}});

  const std::optional<Pose2D> pose = buffer.poseAt(150LL);

  ASSERT_TRUE(pose.has_value());
  EXPECT_NEAR(std::abs(pose->yaw), kPi, 1e-12);
}

TEST(OdometryBuffer2DTest, RejectsTimestampOutsideBufferedSamples)
{
  OdometryBuffer2D buffer;
  buffer.add({100LL, {}});
  buffer.add({200LL, {}});

  EXPECT_FALSE(buffer.poseAt(99LL).has_value());
  EXPECT_FALSE(buffer.poseAt(201LL).has_value());
}

TEST(OdometryBuffer2DTest, RejectsInterpolationAcrossLargeGap)
{
  OdometryBuffer2D buffer({5000000000LL, 100LL});
  buffer.add({100LL, {}});
  buffer.add({300LL, {}});

  EXPECT_FALSE(buffer.poseAt(200LL).has_value());
}

TEST(OdometryBuffer2DTest, ReplacesDuplicateTimestampAndKeepsOrder)
{
  OdometryBuffer2D buffer;
  buffer.add({300LL, {3.0, 0.0, 0.0}});
  buffer.add({100LL, {1.0, 0.0, 0.0}});
  buffer.add({300LL, {4.0, 0.0, 0.0}});

  ASSERT_EQ(buffer.oldestTimestamp(), 100LL);
  ASSERT_EQ(buffer.newestTimestamp(), 300LL);
  ASSERT_TRUE(buffer.poseAt(300LL).has_value());
  EXPECT_DOUBLE_EQ(buffer.poseAt(300LL)->x, 4.0);
}

}  // namespace
}  // namespace p03_slam_core
