#include "p03_slam_core/odometry_buffer2d.hpp"

#include <algorithm>
#include <stdexcept>

#include "p03_slam_core/pose2d.hpp"

namespace p03_slam_core
{

OdometryBuffer2D::OdometryBuffer2D(OdometryBufferConfig config)
: config_(config)
{
  if (config_.buffer_duration_nanoseconds <= 0) {
    throw std::invalid_argument("Odometry buffer duration must be positive.");
  }
  if (config_.max_interpolation_gap_nanoseconds <= 0) {
    throw std::invalid_argument("Maximum odometry interpolation gap must be positive.");
  }
}

void OdometryBuffer2D::add(const Odometry2D & odometry)
{
  const auto insertion_position = std::lower_bound(
    samples_.begin(), samples_.end(), odometry.timestamp_nanoseconds,
    [](const Odometry2D & sample, std::int64_t timestamp) {
      return sample.timestamp_nanoseconds < timestamp;
    });

  if (
    insertion_position != samples_.end() &&
    insertion_position->timestamp_nanoseconds == odometry.timestamp_nanoseconds)
  {
    *insertion_position = odometry;
  } else {
    samples_.insert(insertion_position, odometry);
  }

  const std::int64_t oldest_retained_timestamp =
    samples_.back().timestamp_nanoseconds - config_.buffer_duration_nanoseconds;
  while (
    !samples_.empty() &&
    samples_.front().timestamp_nanoseconds < oldest_retained_timestamp)
  {
    samples_.pop_front();
  }
}

std::optional<Pose2D> OdometryBuffer2D::poseAt(
  std::int64_t timestamp_nanoseconds) const noexcept
{
  if (
    samples_.empty() || timestamp_nanoseconds < samples_.front().timestamp_nanoseconds ||
    timestamp_nanoseconds > samples_.back().timestamp_nanoseconds)
  {
    return std::nullopt;
  }

  const auto right = std::lower_bound(
    samples_.begin(), samples_.end(), timestamp_nanoseconds,
    [](const Odometry2D & sample, std::int64_t timestamp) {
      return sample.timestamp_nanoseconds < timestamp;
    });

  if (right->timestamp_nanoseconds == timestamp_nanoseconds) {
    return right->odom_from_base;
  }

  const auto left = std::prev(right);
  const std::int64_t interval_nanoseconds =
    right->timestamp_nanoseconds - left->timestamp_nanoseconds;
  if (interval_nanoseconds > config_.max_interpolation_gap_nanoseconds) {
    return std::nullopt;
  }

  const double interpolation_ratio = static_cast<double>(
    timestamp_nanoseconds - left->timestamp_nanoseconds) /
    static_cast<double>(interval_nanoseconds);
  const double yaw_difference = normalizeAngle(
    right->odom_from_base.yaw - left->odom_from_base.yaw);

  return Pose2D{
    left->odom_from_base.x +
    interpolation_ratio * (right->odom_from_base.x - left->odom_from_base.x),
    left->odom_from_base.y +
    interpolation_ratio * (right->odom_from_base.y - left->odom_from_base.y),
    normalizeAngle(left->odom_from_base.yaw + interpolation_ratio * yaw_difference)
  };
}

std::optional<std::int64_t> OdometryBuffer2D::oldestTimestamp() const noexcept
{
  if (samples_.empty()) {
    return std::nullopt;
  }
  return samples_.front().timestamp_nanoseconds;
}

std::optional<std::int64_t> OdometryBuffer2D::newestTimestamp() const noexcept
{
  if (samples_.empty()) {
    return std::nullopt;
  }
  return samples_.back().timestamp_nanoseconds;
}

}  // namespace p03_slam_core
