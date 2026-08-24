#pragma once

#include <cstdint>
#include <deque>
#include <optional>

#include "p03_slam_core/odometry2d.hpp"

namespace p03_slam_core
{

// Defines how much timestamped odometry is retained and may be interpolated.
struct OdometryBufferConfig
{
  std::int64_t buffer_duration_nanoseconds{5000000000LL};
  std::int64_t max_interpolation_gap_nanoseconds{200000000LL};
};

// Stores recent odometry and interpolates a pose at a sensor timestamp.
class OdometryBuffer2D
{
public:
  explicit OdometryBuffer2D(OdometryBufferConfig config = {});

  // Adds an odometry sample while preserving timestamp order.
  void add(const Odometry2D & odometry);

  // Returns the interpolated pose, or no value when the timestamp is unavailable.
  [[nodiscard]] std::optional<Pose2D> poseAt(
    std::int64_t timestamp_nanoseconds) const noexcept;

  [[nodiscard]] std::optional<std::int64_t> oldestTimestamp() const noexcept;
  [[nodiscard]] std::optional<std::int64_t> newestTimestamp() const noexcept;

private:
  OdometryBufferConfig config_;
  std::deque<Odometry2D> samples_;
};

}  // namespace p03_slam_core
