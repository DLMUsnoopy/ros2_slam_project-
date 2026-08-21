#include "p03_slam_core/point2d.hpp"

#include <cmath>

namespace p03_slam_core
{

Point2D transformPoint(
  const Pose2D & parent_from_local,
  const Point2D & point_local) noexcept
{
  const double cosine = std::cos(parent_from_local.yaw);
  const double sine = std::sin(parent_from_local.yaw);

  return {
    parent_from_local.x + cosine * point_local.x - sine * point_local.y,
    parent_from_local.y + sine * point_local.x + cosine * point_local.y
  };
}

}  // namespace p03_slam_core
