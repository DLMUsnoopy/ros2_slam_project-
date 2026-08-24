#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>

#include "p03_slam_core/laser_scan2d.hpp"
#include "p03_slam_core/likelihood_field2d.hpp"
#include "p03_slam_core/likelihood_field_matcher2d.hpp"
#include "p03_slam_core/occupancy_grid2d.hpp"
#include "p03_slam_core/occupancy_grid_updater.hpp"
#include "p03_slam_core/pose2d.hpp"

namespace p03_slam_core
{

// Groups the configuration of every frontend component.
struct SlamFrontendConfig
{
  OccupancyGridConfig occupancy_grid;
  InverseSensorModelConfig inverse_sensor_model;
  LikelihoodFieldConfig likelihood_field;
  LikelihoodFieldMatcherConfig matcher;
  double correction_gain{1.0};
  double correction_deadband_translation{0.0};
  double correction_deadband_rotation{0.0};
  std::size_t bootstrap_scans{1};
  double stationary_translation_threshold{0.0};
  double stationary_rotation_threshold{0.0};
  std::size_t stationary_map_update_interval{1};
  std::size_t likelihood_update_interval{1};
  double maximum_angular_velocity_for_matching{
    std::numeric_limits<double>::infinity()};
};

// Describes the pose correction produced from one laser scan.
struct SlamUpdate2D
{
  Pose2D map_from_lidar;
  Pose2D map_from_odom;
  double score{0.0};
  double prior_score{0.0};
  std::size_t matched_point_count{0};
  bool scan_match_attempted{false};
  bool scan_match_accepted{false};
  bool scan_matched{false};
};

// Coordinates scan matching and occupancy-grid updates.
class SlamFrontend2D
{
public:
  explicit SlamFrontend2D(const SlamFrontendConfig & config);

  // Processes one scan using its odometry-based sensor pose.
  [[nodiscard]] SlamUpdate2D processScan(
    const LaserScan2D & scan,
    const Pose2D & odom_from_lidar);

  // Provides the current map for publication or inspection.
  [[nodiscard]] const OccupancyGrid2D & occupancyGrid() const noexcept;
  [[nodiscard]] bool isInitialized() const noexcept;

private:
  // Stores the occupancy-grid map.
  OccupancyGrid2D occupancy_grid_;

  // Inserts laser scans into the occupancy grid.
  OccupancyGridUpdater occupancy_grid_updater_;

  // Stores the obstacle-distance field used by scan matching.
  LikelihoodField2D likelihood_field_;
  LikelihoodFieldMatcher2D matcher_;
  double correction_gain_{1.0};
  double correction_deadband_translation_{0.0};
  double correction_deadband_rotation_{0.0};
  std::size_t bootstrap_scans_{1};
  double stationary_translation_threshold_{0.0};
  double stationary_rotation_threshold_{0.0};
  std::size_t stationary_map_update_interval_{1};
  std::size_t likelihood_update_interval_{1};
  double maximum_angular_velocity_for_matching_{
    std::numeric_limits<double>::infinity()};
  Pose2D map_from_odom_;
  Pose2D last_odom_from_lidar_;
  std::int64_t last_scan_timestamp_nanoseconds_{0};
  std::size_t inserted_scan_count_{0};
  std::size_t stationary_scan_count_{0};
  std::size_t scans_since_likelihood_update_{0};
  bool have_last_odometry_{false};
  bool likelihood_dirty_{false};
  bool initialized_{false};
};

}  // namespace p03_slam_core
