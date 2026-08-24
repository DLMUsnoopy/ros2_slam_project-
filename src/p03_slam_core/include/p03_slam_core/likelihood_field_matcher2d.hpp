#pragma once

#include <cstddef>
#include <vector>

#include "p03_slam_core/laser_scan2d.hpp"
#include "p03_slam_core/likelihood_field2d.hpp"
#include "p03_slam_core/occupancy_grid2d.hpp"
#include "p03_slam_core/point2d.hpp"
#include "p03_slam_core/pose2d.hpp"

namespace p03_slam_core
{

// Controls how quickly the scan score decreases away from obstacles.
struct LikelihoodFieldMatcherConfig
{
  double standard_deviation{0.2};
  double linear_search_window{0.2};
  double angular_search_window{0.1};
  double linear_step{0.05};
  double angular_step{0.025};
  double fine_linear_search_window{0.0};
  double fine_angular_search_window{0.0};
  double fine_linear_step{0.02};
  double fine_angular_step{0.01};
  double min_score{0.0};
  double min_score_improvement{0.0};
  double maximum_range{30.0};
  std::size_t min_matched_points{1};
  std::size_t beam_stride{1};
};

// Contains the best pose found by scan matching and its score.
struct ScanMatchResult2D
{
  Pose2D map_from_lidar;
  double score{0.0};
  double prior_score{0.0};
  std::size_t matched_point_count{0};
  bool accepted{false};
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

  // Searches around an initial pose and returns the highest-scoring candidate.
  [[nodiscard]] ScanMatchResult2D match(
    const OccupancyGrid2D & grid,
    const LikelihoodField2D & field,
    const LaserScan2D & scan,
    const Pose2D & initial_map_from_lidar) const noexcept;

private:
  // Converts valid, downsampled ranges into lidar-frame endpoints once per scan.
  [[nodiscard]] std::vector<Point2D> scanEndpoints(const LaserScan2D & scan) const;

  // Scores endpoints that were precomputed before the candidate search.
  [[nodiscard]] double scoreEndpoints(
    const OccupancyGrid2D & grid,
    const LikelihoodField2D & field,
    const std::vector<Point2D> & endpoints,
    const Pose2D & map_from_lidar,
    std::size_t & matched_point_count) const noexcept;

  // Searches one regular x-y-yaw window around a center pose.
  [[nodiscard]] ScanMatchResult2D search(
    const OccupancyGrid2D & grid,
    const LikelihoodField2D & field,
    const std::vector<Point2D> & endpoints,
    const Pose2D & center,
    double linear_window,
    double angular_window,
    double linear_step,
    double angular_step) const noexcept;

  LikelihoodFieldMatcherConfig config_;
};

}  // namespace p03_slam_core
