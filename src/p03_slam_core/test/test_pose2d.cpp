#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "p03_slam_core/pose2d.hpp"

namespace p03_slam_core
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kTolerance = 1e-12;

TEST(NormalizeAngle, KeepsAnglesInsideTheTargetInterval)
{
  EXPECT_NEAR(normalizeAngle(0.0), 0.0, kTolerance);
  EXPECT_NEAR(normalizeAngle(0.5 * kPi), 0.5 * kPi, kTolerance);
  EXPECT_NEAR(normalizeAngle(-0.5 * kPi), -0.5 * kPi, kTolerance);
}

TEST(NormalizeAngle, WrapsWholeTurns)
{
  EXPECT_NEAR(normalizeAngle(2.0 * kPi), 0.0, kTolerance);
  EXPECT_NEAR(normalizeAngle(-2.0 * kPi), 0.0, kTolerance);
  EXPECT_NEAR(normalizeAngle(3.0 * kPi), -kPi, kTolerance);
}

TEST(NormalizeAngle, UsesNegativePiAtTheBoundary)
{
  EXPECT_NEAR(normalizeAngle(kPi), -kPi, kTolerance);
  EXPECT_NEAR(normalizeAngle(-kPi), -kPi, kTolerance);
}

TEST(NormalizeAngle, ReturnsNanForNonFiniteInput)
{
  EXPECT_TRUE(std::isnan(normalizeAngle(std::numeric_limits<double>::infinity())));
  EXPECT_TRUE(std::isnan(normalizeAngle(-std::numeric_limits<double>::infinity())));
  EXPECT_TRUE(std::isnan(normalizeAngle(std::numeric_limits<double>::quiet_NaN())));
}

TEST(ComposePose, AddsTranslationWithoutRotation)
{
  const Pose2D first{1.0, 2.0, 0.0};
  const Pose2D second{3.0, 4.0, 0.0};

  const Pose2D result = compose(first, second);

  EXPECT_NEAR(result.x, 4.0, kTolerance);
  EXPECT_NEAR(result.y, 6.0, kTolerance);
  EXPECT_NEAR(result.yaw, 0.0, kTolerance);
}

TEST(ComposePose, RotatesLocalTranslationIntoTheParentFrame)
{
  const Pose2D parent{1.0, 2.0, 0.5 * kPi};
  const Pose2D local{1.0, 0.0, 0.0};

  const Pose2D result = compose(parent, local);

  EXPECT_NEAR(result.x, 1.0, kTolerance);
  EXPECT_NEAR(result.y, 3.0, kTolerance);
  EXPECT_NEAR(result.yaw, 0.5 * kPi, kTolerance);
}

TEST(ComposePose, PreservesTransformOrder)
{
  const Pose2D first{1.0, 0.0, 0.5 * kPi};
  const Pose2D second{1.0, 0.0, 0.0};

  const Pose2D first_then_second = compose(first, second);
  const Pose2D second_then_first = compose(second, first);

  EXPECT_NEAR(first_then_second.x, 1.0, kTolerance);
  EXPECT_NEAR(first_then_second.y, 1.0, kTolerance);
  EXPECT_NEAR(second_then_first.x, 2.0, kTolerance);
  EXPECT_NEAR(second_then_first.y, 0.0, kTolerance);
}

TEST(ComposePose, NormalizesTheResultYaw)
{
  const Pose2D first{0.0, 0.0, 0.75 * kPi};
  const Pose2D second{0.0, 0.0, 0.75 * kPi};

  const Pose2D result = compose(first, second);

  EXPECT_NEAR(result.yaw, -0.5 * kPi, kTolerance);
}

TEST(InversePose, KeepsTheIdentityPoseUnchanged)
{
  const Pose2D result = inverse(Pose2D{});

  EXPECT_NEAR(result.x, 0.0, kTolerance);
  EXPECT_NEAR(result.y, 0.0, kTolerance);
  EXPECT_NEAR(result.yaw, 0.0, kTolerance);
}

TEST(InversePose, RotatesTheNegatedTranslation)
{
  const Pose2D pose{1.0, 2.0, 0.5 * kPi};

  const Pose2D result = inverse(pose);

  EXPECT_NEAR(result.x, -2.0, kTolerance);
  EXPECT_NEAR(result.y, 1.0, kTolerance);
  EXPECT_NEAR(result.yaw, -0.5 * kPi, kTolerance);
}

TEST(InversePose, ProducesIdentityFromBothCompositionOrders)
{
  const Pose2D pose{1.25, -0.75, 0.42};
  const Pose2D pose_inverse = inverse(pose);

  const Pose2D left_identity = compose(pose, pose_inverse);
  const Pose2D right_identity = compose(pose_inverse, pose);

  EXPECT_NEAR(left_identity.x, 0.0, kTolerance);
  EXPECT_NEAR(left_identity.y, 0.0, kTolerance);
  EXPECT_NEAR(left_identity.yaw, 0.0, kTolerance);
  EXPECT_NEAR(right_identity.x, 0.0, kTolerance);
  EXPECT_NEAR(right_identity.y, 0.0, kTolerance);
  EXPECT_NEAR(right_identity.yaw, 0.0, kTolerance);
}

}  // namespace
}  // namespace p03_slam_core
