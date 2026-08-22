#include "p03_slam_core/occupancy_grid2d.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace p03_slam_core
{

// Initializes configuration and allocates storage for every grid cell.
OccupancyGrid2D::OccupancyGrid2D(OccupancyGridConfig config)
: config_(std::move(config)),
  log_odds_(config_.width * config_.height, 0.0F),
  observed_(config_.width * config_.height, 0U)
{
}

// Returns read-only access to the grid configuration.
const OccupancyGridConfig & OccupancyGrid2D::config() const noexcept
{
  return config_;
}

// Returns the total number of cells in the grid.
std::size_t OccupancyGrid2D::cellCount() const noexcept
{
  return log_odds_.size();
}

// Checks whether a two-dimensional index lies inside the grid.
bool OccupancyGrid2D::contains(const GridIndex & index) const noexcept
{
  return index.x >= 0 && index.y >= 0 &&
         static_cast<std::size_t>(index.x) < config_.width &&
         static_cast<std::size_t>(index.y) < config_.height;
}

// Converts a world position into a valid grid index.
std::optional<GridIndex> OccupancyGrid2D::worldToGrid(
  const Point2D & point_world) const noexcept
{
  // floor maps negative coordinates downward instead of truncating them toward zero.
  const GridIndex index{
    static_cast<int>(std::floor((point_world.x - config_.origin.x) / config_.resolution)),
    static_cast<int>(std::floor((point_world.y - config_.origin.y) / config_.resolution))
  };

  if (!contains(index)) {
    return std::nullopt;
  }
  return index;
}

// Returns the accumulated occupancy evidence for a cell.
float OccupancyGrid2D::logOdds(const GridIndex & index) const noexcept
{
  return log_odds_[linearIndex(index)];
}

// Reports whether a cell has received any sensor evidence.
bool OccupancyGrid2D::isObserved(const GridIndex & index) const noexcept
{
  return observed_[linearIndex(index)] != 0U;
}

// Adds and clamps occupancy evidence, then marks the cell as observed.
void OccupancyGrid2D::addLogOdds(const GridIndex & index, const float evidence) noexcept
{
  // Converts the two-dimensional index into the storage position.
  const std::size_t cell = linearIndex(index);
  log_odds_[cell] = std::clamp(
    log_odds_[cell] + evidence,
    config_.min_log_odds,
    config_.max_log_odds);
  observed_[cell] = 1U;
}

// Converts a two-dimensional index into a row-major storage offset.
std::size_t OccupancyGrid2D::linearIndex(const GridIndex & index) const noexcept
{
  return static_cast<std::size_t>(index.y) * config_.width + static_cast<std::size_t>(index.x);
}

}  // namespace p03_slam_core
