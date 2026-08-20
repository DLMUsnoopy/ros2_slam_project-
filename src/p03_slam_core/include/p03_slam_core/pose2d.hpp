#pragma once

namespace p03_slam_core
{

/// Represents a rigid-body pose in a two-dimensional coordinate frame.
struct Pose2D
{
  double x{0.0};    ///< Position along the x-axis, in meters.
  double y{0.0};    ///< along the y-axis meters.
  double yaw{0.0};  ///< Rotation around the z-axis, in radians.
};

/// Normalizes a finite angle to the half-open interval [-pi, pi).
/// Returns NaN when the input is not finite.
[[nodiscard]] double normalizeAngle(double angle) noexcept;

/// Composes two poses using the transform order result = first * second.
[[nodiscard]] Pose2D compose(const Pose2D & first, const Pose2D & second) noexcept;

/// Returns the inverse rigid-body transform of a pose.
[[nodiscard]] Pose2D inverse(const Pose2D & pose) noexcept;

}  // namespace p03_slam_core
