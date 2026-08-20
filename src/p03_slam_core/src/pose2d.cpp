#include "p03_slam_core/pose2d.hpp"

#include <cmath>
#include <limits>

namespace p03_slam_core
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

}  // namespace

double normalizeAngle(const double angle) noexcept
{
  if (!std::isfinite(angle)) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  double normalized = std::remainder(angle, kTwoPi);
  if (normalized >= kPi) {
    normalized -= kTwoPi;
  }

  return normalized == 0.0 ? 0.0 : normalized;
}

Pose2D compose(const Pose2D & first, const Pose2D & second) noexcept
{
  const double cosine = std::cos(first.yaw);
  const double sine = std::sin(first.yaw);

  return {
    first.x + cosine * second.x - sine * second.y,
    first.y + sine * second.x + cosine * second.y,
    normalizeAngle(first.yaw + second.yaw)
  };
}

Pose2D inverse(const Pose2D & pose) noexcept
{
  const double cosine = std::cos(pose.yaw);
  const double sine = std::sin(pose.yaw);

  return {
    -cosine * pose.x - sine * pose.y,
    sine * pose.x - cosine * pose.y,
    normalizeAngle(-pose.yaw)
  };
}

}  // namespace p03_slam_core
