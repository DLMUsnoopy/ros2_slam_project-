#include "p03_slam_core/likelihood_field_matcher2d.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

#include "p03_slam_core/point2d.hpp"

namespace p03_slam_core
{

LikelihoodFieldMatcher2D::LikelihoodFieldMatcher2D(LikelihoodFieldMatcherConfig config)
: config_(config)
{
}

std::vector<Point2D> LikelihoodFieldMatcher2D::scanEndpoints(
  const LaserScan2D & scan) const
{
  const std::size_t beam_stride = std::max<std::size_t>(config_.beam_stride, 1U);
  std::vector<Point2D> endpoints;
  endpoints.reserve(scan.ranges.size() / beam_stride + 1U);

  for (std::size_t beam = 0; beam < scan.ranges.size(); beam += beam_stride) {
    const double range = static_cast<double>(scan.ranges[beam]);
    if (
      !std::isfinite(range) || range < scan.range_min || range >= scan.range_max ||
      range > config_.maximum_range)
    {
      continue;
    }

    const double angle = scan.angle_min + static_cast<double>(beam) * scan.angle_increment;
    endpoints.push_back({range * std::cos(angle), range * std::sin(angle)});
  }
  return endpoints;
}

double LikelihoodFieldMatcher2D::scoreEndpoints(
  const OccupancyGrid2D & grid,
  const LikelihoodField2D & field,
  const std::vector<Point2D> & endpoints,
  const Pose2D & map_from_lidar,
  std::size_t & matched_point_count) const noexcept
{
  double score_sum = 0.0;
  matched_point_count = 0U;
  const double variance_factor = 2.0 * config_.standard_deviation *
    config_.standard_deviation;

  for (const Point2D & endpoint_in_lidar : endpoints) {
    const Point2D endpoint_in_map = transformPoint(map_from_lidar, endpoint_in_lidar);
    const std::optional<GridIndex> endpoint_index = grid.worldToGrid(endpoint_in_map);
    if (!endpoint_index) {
      continue;
    }

    ++matched_point_count;
    const double distance = static_cast<double>(field.obstacleDistance(*endpoint_index));
    score_sum += std::exp(-(distance * distance) / variance_factor);
  }

  if (endpoints.empty()) {
    return 0.0;
  }
  return score_sum / static_cast<double>(endpoints.size());
}

// Transforms every valid scan endpoint and averages its obstacle likelihood.
double LikelihoodFieldMatcher2D::scorePose(
  const OccupancyGrid2D & grid,
  const LikelihoodField2D & field,
  const LaserScan2D & scan,
  const Pose2D & map_from_lidar) const noexcept
{
  const std::vector<Point2D> endpoints = scanEndpoints(scan);
  std::size_t matched_point_count = 0U;
  return scoreEndpoints(
    grid, field, endpoints, map_from_lidar, matched_point_count);
}

ScanMatchResult2D LikelihoodFieldMatcher2D::search(
  const OccupancyGrid2D & grid,
  const LikelihoodField2D & field,
  const std::vector<Point2D> & endpoints,
  const Pose2D & center,
  const double linear_window,
  const double angular_window,
  const double linear_step,
  const double angular_step) const noexcept
{
  ScanMatchResult2D best_result;
  best_result.map_from_lidar = center;
  best_result.score = -std::numeric_limits<double>::infinity();
  double best_offset_cost = std::numeric_limits<double>::infinity();

  const int linear_steps = linear_step > 0.0 ?
    static_cast<int>(std::floor(linear_window / linear_step)) : 0;
  const int angular_steps = angular_step > 0.0 ?
    static_cast<int>(std::floor(angular_window / angular_step)) : 0;

  for (int yaw_step = -angular_steps; yaw_step <= angular_steps; ++yaw_step) {
    for (int y_step = -linear_steps; y_step <= linear_steps; ++y_step) {
      for (int x_step = -linear_steps; x_step <= linear_steps; ++x_step) {
        // Defines one candidate around the initial x, y, and yaw values.
        Pose2D candidate = center;
        candidate.x += static_cast<double>(x_step) * linear_step;
        candidate.y += static_cast<double>(y_step) * linear_step;
        candidate.yaw = normalizeAngle(
          candidate.yaw + static_cast<double>(yaw_step) * angular_step);

        std::size_t matched_point_count = 0U;
        const double candidate_score = scoreEndpoints(
          grid, field, endpoints, candidate, matched_point_count);
        const double offset_cost = static_cast<double>(
          x_step * x_step + y_step * y_step + yaw_step * yaw_step);
        constexpr double kScoreTieEpsilon = 1e-9;
        if (
          candidate_score > best_result.score + kScoreTieEpsilon ||
          (std::abs(candidate_score - best_result.score) <= kScoreTieEpsilon &&
          offset_cost < best_offset_cost))
        {
          best_result.map_from_lidar = candidate;
          best_result.score = candidate_score;
          best_result.matched_point_count = matched_point_count;
          best_offset_cost = offset_cost;
        }
      }
    }
  }

  if (!std::isfinite(best_result.score)) {
    best_result.score = 0.0;
  }
  return best_result;
}

// Runs a coarse search, refines its result, and validates the improvement.
ScanMatchResult2D LikelihoodFieldMatcher2D::match(
  const OccupancyGrid2D & grid,
  const LikelihoodField2D & field,
  const LaserScan2D & scan,
  const Pose2D & initial_map_from_lidar) const noexcept
{
  const std::vector<Point2D> endpoints = scanEndpoints(scan);
  if (endpoints.size() < config_.min_matched_points) {
    return {initial_map_from_lidar, 0.0, 0.0, endpoints.size(), false};
  }

  std::size_t prior_matched_point_count = 0U;
  const double prior_score = scoreEndpoints(
    grid, field, endpoints, initial_map_from_lidar, prior_matched_point_count);

  const ScanMatchResult2D coarse = search(
    grid, field, endpoints, initial_map_from_lidar,
    config_.linear_search_window, config_.angular_search_window,
    config_.linear_step, config_.angular_step);
  ScanMatchResult2D fine = search(
    grid, field, endpoints, coarse.map_from_lidar,
    config_.fine_linear_search_window, config_.fine_angular_search_window,
    config_.fine_linear_step, config_.fine_angular_step);

  fine.prior_score = prior_score;
  fine.accepted = fine.score >= config_.min_score &&
    fine.score >= prior_score + config_.min_score_improvement &&
    fine.matched_point_count >= config_.min_matched_points;
  return fine;
}

}  // namespace p03_slam_core
