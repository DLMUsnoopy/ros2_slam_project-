#include "p03_slam_core/likelihood_field2d.hpp"

#include <array>
#include <functional>
#include <queue>
#include <utility>

namespace p03_slam_core
{

// Stores the configuration used when rebuilding the distance field.
LikelihoodField2D::LikelihoodField2D(LikelihoodFieldConfig config)
: config_(config)
{
}

// Builds an eight-connected distance field from all occupied cells.
void LikelihoodField2D::rebuild(const OccupancyGrid2D & grid)
{
  using QueueEntry = std::pair<float, std::size_t>;
  std::priority_queue<
    QueueEntry,
    std::vector<QueueEntry>,
    std::greater<QueueEntry>> open_cells;

  width_ = grid.config().width;
  // Initializes every cell with the configured distance limit.
  obstacle_distances_.assign(grid.cellCount(), config_.max_obstacle_distance);

  for (std::size_t y = 0; y < grid.config().height; ++y) {
    for (std::size_t x = 0; x < grid.config().width; ++x) {
      const GridIndex index{static_cast<int>(x), static_cast<int>(y)};
      if (grid.isObserved(index) &&
        grid.logOdds(index) > config_.occupied_log_odds_threshold)
      {
        const std::size_t cell = linearIndex(index);
        obstacle_distances_[cell] = 0.0F;
        // Adds each obstacle as a zero-distance search source.
        open_cells.emplace(0.0F, cell);
      }
    }
  }

  constexpr float kSqrtTwo = 1.41421356237F;
  constexpr std::array<GridIndex, 8> kNeighborOffsets{{
    {-1, -1}, {0, -1}, {1, -1},
    {-1, 0}, {1, 0},
    {-1, 1}, {0, 1}, {1, 1}
  }};

  while (!open_cells.empty()) {
    // Structured binding gives names to both values stored in the queue entry.
    const auto [current_distance, current_cell] = open_cells.top();
    open_cells.pop();

    // Ignores a queue entry when a shorter route was already found.
    if (current_distance > obstacle_distances_[current_cell]) {
      continue;
    }

    const GridIndex current{
      static_cast<int>(current_cell % width_),
      static_cast<int>(current_cell / width_)
    };

    for (const GridIndex & offset : kNeighborOffsets) {
      const GridIndex neighbor{current.x + offset.x, current.y + offset.y};
      if (!grid.contains(neighbor)) {
        continue;
      }

      const bool is_diagonal = offset.x != 0 && offset.y != 0;
      // A diagonal move covers sqrt(2) times the distance of a straight move.
      const float step_distance = static_cast<float>(grid.config().resolution) *
        (is_diagonal ? kSqrtTwo : 1.0F);
      const float candidate_distance = current_distance + step_distance;
      const std::size_t neighbor_cell = linearIndex(neighbor);

      if (candidate_distance < obstacle_distances_[neighbor_cell]) {
        obstacle_distances_[neighbor_cell] = candidate_distance;
        open_cells.emplace(candidate_distance, neighbor_cell);
      }
    }
  }
}

// Returns the stored obstacle distance for one cell.
float LikelihoodField2D::obstacleDistance(const GridIndex & index) const noexcept
{
  return obstacle_distances_[linearIndex(index)];
}

// Returns the number of cells currently stored in the field.
std::size_t LikelihoodField2D::cellCount() const noexcept
{
  return obstacle_distances_.size();
}

// Converts a two-dimensional index into a row-major storage offset.
std::size_t LikelihoodField2D::linearIndex(const GridIndex & index) const noexcept
{
  return static_cast<std::size_t>(index.y) * width_ + static_cast<std::size_t>(index.x);
}

}  // namespace p03_slam_core
