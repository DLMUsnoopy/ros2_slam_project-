#include "p03_slam_evaluation/trajectory_evaluator2d.hpp"

#include <algorithm>
#include <cmath>

#include "p03_slam_core/pose2d.hpp"

namespace p03_slam_evaluation
{

TrajectoryEvaluation2D TrajectoryEvaluator2D::addSample(
  const p03_slam_core::Pose2D & estimated_pose,
  const p03_slam_core::Pose2D & ground_truth_pose)
{
  if (!estimate_from_ground_truth_) {
    estimate_from_ground_truth_ = p03_slam_core::compose(
      estimated_pose, p03_slam_core::inverse(ground_truth_pose));
  }

  const p03_slam_core::Pose2D aligned_ground_truth = p03_slam_core::compose(
    *estimate_from_ground_truth_, ground_truth_pose);
  const double x_error = estimated_pose.x - aligned_ground_truth.x;
  const double y_error = estimated_pose.y - aligned_ground_truth.y;
  const double translation_error = std::hypot(x_error, y_error);
  const double yaw_error = std::abs(p03_slam_core::normalizeAngle(
      estimated_pose.yaw - aligned_ground_truth.yaw));

  ++metrics_.sample_count;
  metrics_.latest_translation_error = translation_error;
  metrics_.latest_yaw_error = yaw_error;
  metrics_.maximum_translation_error = std::max(
    metrics_.maximum_translation_error, translation_error);
  metrics_.maximum_yaw_error = std::max(metrics_.maximum_yaw_error, yaw_error);

  squared_translation_error_sum_ += translation_error * translation_error;
  squared_yaw_error_sum_ += yaw_error * yaw_error;
  metrics_.translation_rmse = std::sqrt(
    squared_translation_error_sum_ / static_cast<double>(metrics_.sample_count));
  metrics_.yaw_rmse = std::sqrt(
    squared_yaw_error_sum_ / static_cast<double>(metrics_.sample_count));

  return {aligned_ground_truth, metrics_};
}

const TrajectoryMetrics2D & TrajectoryEvaluator2D::metrics() const noexcept
{
  return metrics_;
}

}  // namespace p03_slam_evaluation
