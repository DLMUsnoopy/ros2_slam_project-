#pragma once

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
};

// Describes the pose correction produced from one laser scan.
struct SlamUpdate2D
{
  Pose2D map_from_lidar;
  Pose2D map_from_odom;
  double score{0.0};
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
  Pose2D map_from_odom_;
  bool initialized_{false};
};

}  // namespace p03_slam_core
