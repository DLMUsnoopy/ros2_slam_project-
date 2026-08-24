#pragma once

#include <cstddef>
#include <optional>

#include "p03_slam_core/pose2d.hpp"

namespace p03_slam_evaluation
{

// Contains the latest and accumulated trajectory errors.
struct TrajectoryMetrics2D
{
  std::size_t sample_count{0};
  double latest_translation_error{0.0};
  double latest_yaw_error{0.0};
  double translation_rmse{0.0};
  double yaw_rmse{0.0};
  double maximum_translation_error{0.0};
  double maximum_yaw_error{0.0};
};

// Contains one aligned ground-truth pose and the updated metrics.
struct TrajectoryEvaluation2D
{
  p03_slam_core::Pose2D aligned_ground_truth;
  TrajectoryMetrics2D metrics;
};

// Aligns ground truth at the first sample and accumulates planar pose errors.
class TrajectoryEvaluator2D
{
public:
  [[nodiscard]] TrajectoryEvaluation2D addSample(
    const p03_slam_core::Pose2D & estimated_pose,
    const p03_slam_core::Pose2D & ground_truth_pose);

  [[nodiscard]] const TrajectoryMetrics2D & metrics() const noexcept;

private:
  std::optional<p03_slam_core::Pose2D> estimate_from_ground_truth_;
  TrajectoryMetrics2D metrics_;
  double squared_translation_error_sum_{0.0};
  double squared_yaw_error_sum_{0.0};
};

}  // namespace p03_slam_evaluation
