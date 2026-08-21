#pragma once

#include "p03_slam_core/pose2d.hpp"

namespace p03_slam_core
{

/// Represents a point in a two-dimensional coordinate frame.
struct Point2D
{
  double x{0.0};  ///< Position along the x-axis, in meters.
  double y{0.0};  ///< Position along the y-axis, in meters.
};

/// Transforms a point from a local frame into its parent frame.
[[nodiscard]] Point2D transformPoint(
  const Pose2D & parent_from_local,
  const Point2D & point_local) noexcept;

}  // namespace p03_slam_core
