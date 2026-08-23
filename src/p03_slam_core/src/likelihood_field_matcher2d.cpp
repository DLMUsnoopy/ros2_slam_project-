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

  for (std::size_t beam = 0; beam < scan.ranges.size(); ++beam) {    //loop for each scan 
    const double range = static_cast<double>(scan.ranges[beam]);        
    if (!std::isfinite(range) || range < scan.range_min || range >= scan.range_max) {   //filter the beam 
      continue;
    }

    ++valid_beam_count;
    const double angle = scan.angle_min + static_cast<double>(beam) * scan.angle_increment;  //eg beam 2 → angle_min + 2 * increment
    const Point2D endpoint_in_lidar{range * std::cos(angle), range * std::sin(angle)};
    const Point2D endpoint_in_map = transformPoint(map_from_lidar, endpoint_in_lidar);  //lidar to world 
    const std::optional<GridIndex> endpoint_index = grid.worldToGrid(endpoint_in_map);   //world to grid 
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

}  // namespace p03_slam_core
