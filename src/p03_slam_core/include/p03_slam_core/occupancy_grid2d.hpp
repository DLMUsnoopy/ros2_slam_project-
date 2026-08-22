#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "p03_slam_core/point2d.hpp"

namespace p03_slam_core
{

struct GridIndex
{
  // Member values default to zero when no explicit value is provided.
  int x{0};
  int y{0};
};

// Defines the map size, resolution, origin, and log-odds limits.
struct OccupancyGridConfig
{
  std::size_t width{0};
  std::size_t height{0};
  double resolution{0.05};
  Point2D origin;
  float min_log_odds{-4.0F};
  float max_log_odds{4.0F};
};

class OccupancyGrid2D
{
public:
  explicit OccupancyGrid2D(OccupancyGridConfig config);

  /// Returns read-only access to the grid configuration.
  [[nodiscard]] const OccupancyGridConfig & config() const noexcept;
  [[nodiscard]] std::size_t cellCount() const noexcept;
  [[nodiscard]] bool contains(const GridIndex & index) const noexcept;
  [[nodiscard]] std::optional<GridIndex> worldToGrid(
    const Point2D & point_world) const noexcept;
  [[nodiscard]] float logOdds(const GridIndex & index) const noexcept;
  [[nodiscard]] bool isObserved(const GridIndex & index) const noexcept;
  void addLogOdds(const GridIndex & index, float evidence) noexcept;

private:
  [[nodiscard]] std::size_t linearIndex(const GridIndex & index) const noexcept;

  OccupancyGridConfig config_;
  std::vector<float> log_odds_;
  std::vector<std::uint8_t> observed_;
};

}  // namespace p03_slam_core
