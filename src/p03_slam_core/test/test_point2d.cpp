#include <gtest/gtest.h>

#include "p03_slam_core/point2d.hpp"

namespace p03_slam_core
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kTolerance = 1e-12;

TEST(TransformPoint, KeepsPointUnchangedForIdentityPose)
{
  const Point2D point_local{2.0, -1.0};

  const Point2D point_parent = transformPoint(Pose2D{}, point_local);

  EXPECT_NEAR(point_parent.x, 2.0, kTolerance);
  EXPECT_NEAR(point_parent.y, -1.0, kTolerance);
}

TEST(TransformPoint, AppliesTranslation)
{
  const Pose2D parent_from_local{1.0, 3.0, 0.0};
  const Point2D point_local{2.0, -1.0};

  const Point2D point_parent = transformPoint(parent_from_local, point_local);

  EXPECT_NEAR(point_parent.x, 3.0, kTolerance);
  EXPECT_NEAR(point_parent.y, 2.0, kTolerance);
}

TEST(TransformPoint, AppliesRotation)
{
  const Pose2D parent_from_local{0.0, 0.0, 0.5 * kPi};
  const Point2D point_local{2.0, 0.0};

  const Point2D point_parent = transformPoint(parent_from_local, point_local);

  EXPECT_NEAR(point_parent.x, 0.0, kTolerance);
  EXPECT_NEAR(point_parent.y, 2.0, kTolerance);
}

TEST(TransformPoint, AppliesRotationBeforeTranslation)
{
  const Pose2D map_from_base{1.0, 3.0, 0.5 * kPi};
  const Point2D point_base{2.0, 0.0};

  const Point2D point_map = transformPoint(map_from_base, point_base);

  EXPECT_NEAR(point_map.x, 1.0, kTolerance);
  EXPECT_NEAR(point_map.y, 5.0, kTolerance);
}

}  // namespace
}  // namespace p03_slam_core
