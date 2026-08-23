#include "p03_slam_core/likelihood_field_matcher2d.hpp"

#include <cmath>
#include <cstddef>
#include <optional>

#include "p03_slam_core/point2d.hpp"

namespace p03_slam_core
{

LikelihoodFieldMatcher2D::LikelihoodFieldMatcher2D(LikelihoodFieldMatcherConfig config)
: config_(config)
{
}

// Transforms every valid scan endpoint and averages its obstacle likelihood.
double LikelihoodFieldMatcher2D::scorePose(
  const OccupancyGrid2D & grid,
  const LikelihoodField2D & field,
  const LaserScan2D & scan,
  const Pose2D & map_from_lidar) const noexcept
{
  double score_sum = 0.0;
  std::size_t valid_beam_count = 0;
  const double variance_factor = 2.0 * config_.standard_deviation *
    config_.standard_deviation;

  // Evaluates each beam that represents a valid obstacle hit.
  for (std::size_t beam = 0; beam < scan.ranges.size(); ++beam) {
    const double range = static_cast<double>(scan.ranges[beam]);
    if (!std::isfinite(range) || range < scan.range_min || range >= scan.range_max) {
      continue;
    }

    ++valid_beam_count;
    // For example, beam two uses angle_min + 2 * angle_increment.
    const double angle = scan.angle_min + static_cast<double>(beam) * scan.angle_increment;
    const Point2D endpoint_in_lidar{range * std::cos(angle), range * std::sin(angle)};
    const Point2D endpoint_in_map = transformPoint(map_from_lidar, endpoint_in_lidar);
    const std::optional<GridIndex> endpoint_index = grid.worldToGrid(endpoint_in_map);
    if (!endpoint_index) {
      continue;
    }

    const double distance = static_cast<double>(field.obstacleDistance(*endpoint_index));
    score_sum += std::exp(-(distance * distance) / variance_factor);
  }

  if (valid_beam_count == 0U) {
    return 0.0;
  }
  return score_sum / static_cast<double>(valid_beam_count);
}

// Evaluates a regular x-y-yaw grid around the odometry-based initial pose.
ScanMatchResult2D LikelihoodFieldMatcher2D::match(
  const OccupancyGrid2D & grid,
  const LikelihoodField2D & field,
  const LaserScan2D & scan,
  const Pose2D & initial_map_from_lidar) const noexcept
{
  // Uses the odometry-based pose as the best result until a better candidate is found.
  ScanMatchResult2D best_result{
    initial_map_from_lidar,
    scorePose(grid, field, scan, initial_map_from_lidar)
  };

  const int linear_steps = static_cast<int>(
    std::floor(config_.linear_search_window / config_.linear_step));
  const int angular_steps = static_cast<int>(
    std::floor(config_.angular_search_window / config_.angular_step));

  for (int yaw_step = -angular_steps; yaw_step <= angular_steps; ++yaw_step) {
    for (int y_step = -linear_steps; y_step <= linear_steps; ++y_step) {
      for (int x_step = -linear_steps; x_step <= linear_steps; ++x_step) {
        // Defines one candidate around the initial x, y, and yaw values.
        Pose2D candidate = initial_map_from_lidar;
        candidate.x += static_cast<double>(x_step) * config_.linear_step;
        candidate.y += static_cast<double>(y_step) * config_.linear_step;
        candidate.yaw = normalizeAngle(
          candidate.yaw + static_cast<double>(yaw_step) * config_.angular_step);

        const double candidate_score = scorePose(grid, field, scan, candidate);
        if (candidate_score > best_result.score) {
          best_result = {candidate, candidate_score};
        }
      }
    }
  }

  return best_result;
}

}  // namespace p03_slam_core
