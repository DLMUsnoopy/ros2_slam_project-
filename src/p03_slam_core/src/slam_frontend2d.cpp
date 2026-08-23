#include "p03_slam_core/slam_frontend2d.hpp"

namespace p03_slam_core
{

SlamFrontend2D::SlamFrontend2D(const SlamFrontendConfig & config)
: occupancy_grid_(config.occupancy_grid),
  occupancy_grid_updater_(config.inverse_sensor_model),
  likelihood_field_(config.likelihood_field),
  matcher_(config.matcher)
{
}

// Matches each scan after the first one and updates the map and odometry correction.
SlamUpdate2D SlamFrontend2D::processScan(
  const LaserScan2D & scan,
  const Pose2D & odom_from_lidar)
{
  const Pose2D predicted_map_from_lidar = compose(map_from_odom_, odom_from_lidar);

  if (!initialized_) {
    occupancy_grid_updater_.insertScan(
      occupancy_grid_, scan, predicted_map_from_lidar);
    likelihood_field_.rebuild(occupancy_grid_);
    initialized_ = true;
    return {predicted_map_from_lidar, map_from_odom_, 0.0, false};
  }

  const ScanMatchResult2D match_result = matcher_.match(
    occupancy_grid_, likelihood_field_, scan, predicted_map_from_lidar);

  // Computes the correction from the matched pose and the odometry prediction.
  map_from_odom_ = compose(
    match_result.map_from_lidar,
    inverse(odom_from_lidar));

  // Updates the map using the corrected lidar pose.
  occupancy_grid_updater_.insertScan(
    occupancy_grid_, scan, match_result.map_from_lidar);
  likelihood_field_.rebuild(occupancy_grid_);

  return {
    match_result.map_from_lidar,
    map_from_odom_,
    match_result.score,
    true
  };
}

const OccupancyGrid2D & SlamFrontend2D::occupancyGrid() const noexcept
{
  return occupancy_grid_;
}

bool SlamFrontend2D::isInitialized() const noexcept
{
  return initialized_;
}

}  // namespace p03_slam_core
