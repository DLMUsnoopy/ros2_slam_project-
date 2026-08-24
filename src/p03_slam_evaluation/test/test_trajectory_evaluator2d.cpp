#include <gtest/gtest.h>

#include <cmath>

#include "p03_slam_core/pose2d.hpp"
#include "p03_slam_evaluation/trajectory_evaluator2d.hpp"

namespace p03_slam_evaluation
{
namespace
{

TEST(TrajectoryEvaluator2D, RemovesTheInitialCoordinateFrameDifference)
{
  TrajectoryEvaluator2D evaluator;
  const p03_slam_core::Pose2D estimate_from_ground_truth{2.0, -1.0, 0.7};
  const p03_slam_core::Pose2D initial_ground_truth{1.0, 1.0, -0.9};
  const p03_slam_core::Pose2D later_ground_truth{2.5, -0.5, 0.4};

  const auto initial = evaluator.addSample(
    p03_slam_core::compose(estimate_from_ground_truth, initial_ground_truth),
    initial_ground_truth);
  const auto later = evaluator.addSample(
    p03_slam_core::compose(estimate_from_ground_truth, later_ground_truth),
    later_ground_truth);

  EXPECT_NEAR(initial.metrics.latest_translation_error, 0.0, 1e-12);
  EXPECT_NEAR(initial.metrics.latest_yaw_error, 0.0, 1e-12);
  EXPECT_NEAR(later.metrics.latest_translation_error, 0.0, 1e-12);
  EXPECT_NEAR(later.metrics.latest_yaw_error, 0.0, 1e-12);
}

TEST(TrajectoryEvaluator2D, AccumulatesRmseAndMaximumErrors)
{
  TrajectoryEvaluator2D evaluator;

  static_cast<void>(evaluator.addSample({0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}));
  static_cast<void>(evaluator.addSample({1.3, 0.0, 0.1}, {1.0, 0.0, 0.0}));
  const auto result = evaluator.addSample({2.4, 0.0, -0.2}, {2.0, 0.0, 0.0});

  EXPECT_EQ(result.metrics.sample_count, 3U);
  EXPECT_NEAR(result.metrics.latest_translation_error, 0.4, 1e-12);
  EXPECT_NEAR(result.metrics.latest_yaw_error, 0.2, 1e-12);
  EXPECT_NEAR(result.metrics.translation_rmse, std::sqrt(0.25 / 3.0), 1e-12);
  EXPECT_NEAR(result.metrics.yaw_rmse, std::sqrt(0.05 / 3.0), 1e-12);
  EXPECT_NEAR(result.metrics.maximum_translation_error, 0.4, 1e-12);
  EXPECT_NEAR(result.metrics.maximum_yaw_error, 0.2, 1e-12);
}

}  // namespace
}  // namespace p03_slam_evaluation
