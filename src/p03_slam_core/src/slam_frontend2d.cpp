#include "p03_slam_core/slam_frontend2d.hpp"

#include <algorithm>
#include <cmath>

// Controls the SLAM processing pipeline.
namespace p03_slam_core
{

SlamFrontend2D::SlamFrontend2D(const SlamFrontendConfig & config)
: occupancy_grid_(config.occupancy_grid),
  occupancy_grid_updater_(config.inverse_sensor_model),
  likelihood_field_(config.likelihood_field),
  matcher_(config.matcher),
  correction_gain_(std::clamp(config.correction_gain, 0.0, 1.0)),
  correction_deadband_translation_(config.correction_deadband_translation),
  correction_deadband_rotation_(config.correction_deadband_rotation),
  bootstrap_scans_(std::max<std::size_t>(config.bootstrap_scans, 1U)),
  stationary_translation_threshold_(config.stationary_translation_threshold),
  stationary_rotation_threshold_(config.stationary_rotation_threshold),
  stationary_map_update_interval_(std::max<std::size_t>(
      config.stationary_map_update_interval, 1U)),
  likelihood_update_interval_(std::max<std::size_t>(config.likelihood_update_interval, 1U)),
  maximum_angular_velocity_for_matching_(config.maximum_angular_velocity_for_matching)
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
    ++inserted_scan_count_;
    last_odom_from_lidar_ = odom_from_lidar;
    last_scan_timestamp_nanoseconds_ = scan.timestamp_nanoseconds;
    have_last_odometry_ = true;
    initialized_ = true;
    return {
      predicted_map_from_lidar, map_from_odom_, 0.0, 0.0, 0U, false, false, false};
  }

  Pose2D selected_map_from_lidar = predicted_map_from_lidar;
  double score = 0.0;
  double prior_score = 0.0;
  std::size_t matched_point_count = 0U;
  bool scan_match_attempted = false;
  bool scan_match_accepted = false;
  bool correction_applied = false;
  const bool stationary = have_last_odometry_ &&
    std::hypot(
    odom_from_lidar.x - last_odom_from_lidar_.x,
    odom_from_lidar.y - last_odom_from_lidar_.y) < stationary_translation_threshold_ &&
    std::abs(normalizeAngle(odom_from_lidar.yaw - last_odom_from_lidar_.yaw)) <
    stationary_rotation_threshold_;
  const double scan_interval_seconds = static_cast<double>(
    scan.timestamp_nanoseconds - last_scan_timestamp_nanoseconds_) * 1.0e-9;
  const double angular_velocity = scan_interval_seconds > 0.0 ?
    std::abs(normalizeAngle(odom_from_lidar.yaw - last_odom_from_lidar_.yaw)) /
    scan_interval_seconds : std::numeric_limits<double>::infinity();
  const bool matching_motion_is_safe =
    angular_velocity <= maximum_angular_velocity_for_matching_;

  if (
    !stationary && matching_motion_is_safe &&
    inserted_scan_count_ >= bootstrap_scans_)
  {
    if (
      likelihood_dirty_ &&
      scans_since_likelihood_update_ >= likelihood_update_interval_)
    {
      likelihood_field_.rebuild(occupancy_grid_);
      likelihood_dirty_ = false;
      scans_since_likelihood_update_ = 0U;
    }

    const ScanMatchResult2D match_result = matcher_.match(
      occupancy_grid_, likelihood_field_, scan, predicted_map_from_lidar);
    scan_match_attempted = true;
    score = match_result.score;
    prior_score = match_result.prior_score;
    matched_point_count = match_result.matched_point_count;
    scan_match_accepted = match_result.accepted;

    if (match_result.accepted) {
      const double correction_translation = std::hypot(
        match_result.map_from_lidar.x - predicted_map_from_lidar.x,
        match_result.map_from_lidar.y - predicted_map_from_lidar.y);
      const double correction_rotation = normalizeAngle(
        match_result.map_from_lidar.yaw - predicted_map_from_lidar.yaw);

      if (
        correction_translation >= correction_deadband_translation_ ||
        std::abs(correction_rotation) >= correction_deadband_rotation_)
      {
        selected_map_from_lidar.x = predicted_map_from_lidar.x + correction_gain_ *
          (match_result.map_from_lidar.x - predicted_map_from_lidar.x);
        selected_map_from_lidar.y = predicted_map_from_lidar.y + correction_gain_ *
          (match_result.map_from_lidar.y - predicted_map_from_lidar.y);
        selected_map_from_lidar.yaw = normalizeAngle(
          predicted_map_from_lidar.yaw + correction_gain_ * correction_rotation);
        map_from_odom_ = compose(selected_map_from_lidar, inverse(odom_from_lidar));
        correction_applied = true;
      }
    }
  }

  bool insert_scan = inserted_scan_count_ < bootstrap_scans_;
  if (!stationary) {
    stationary_scan_count_ = 0U;
    insert_scan = true;
  } else {
    ++stationary_scan_count_;
    insert_scan = insert_scan ||
      stationary_scan_count_ % stationary_map_update_interval_ == 0U;
  }

  if (insert_scan) {
    occupancy_grid_updater_.insertScan(
      occupancy_grid_, scan, selected_map_from_lidar);
    ++inserted_scan_count_;
    ++scans_since_likelihood_update_;
    likelihood_dirty_ = true;
  }

  last_odom_from_lidar_ = odom_from_lidar;
  last_scan_timestamp_nanoseconds_ = scan.timestamp_nanoseconds;
  have_last_odometry_ = true;

  return {
    selected_map_from_lidar,
    map_from_odom_,
    score,
    prior_score,
    matched_point_count,
    scan_match_attempted,
    scan_match_accepted,
    correction_applied
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
