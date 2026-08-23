#pragma once

#include "p03_slam_core/laser_scan2d.hpp"
#include "p03_slam_core/likelihood_field2d.hpp"
#include "p03_slam_core/occupancy_grid2d.hpp"
#include "p03_slam_core/pose2d.hpp"

namespace p03_slam_core
{

// Controls how quickly the scan score decreases away from obstacles.
struct LikelihoodFieldMatcherConfig
{
  double standard_deviation{0.2};
};

// Scores candidate scan poses against a likelihood field.
class LikelihoodFieldMatcher2D
{
public:
  explicit LikelihoodFieldMatcher2D(LikelihoodFieldMatcherConfig config = {});

  // Returns an average endpoint score in the range [0, 1].
  [[nodiscard]] double scorePose(
    const OccupancyGrid2D & grid,
    const LikelihoodField2D & field,
    const LaserScan2D & scan,
    const Pose2D & map_from_lidar) const noexcept;

private:
  LikelihoodFieldMatcherConfig config_;
};

}  // namespace p03_slam_core
