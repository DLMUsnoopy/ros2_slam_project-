#pragma once

#include <cstddef>
#include <vector>

#include "p03_slam_core/occupancy_grid2d.hpp"

namespace p03_slam_core
{

// Defines which cells are obstacles and how far obstacle distances are stored.
struct LikelihoodFieldConfig
{
  float occupied_log_odds_threshold{0.0F};
  float max_obstacle_distance{2.0F};
};

// Builds from occupied cells and stores each cell's nearest-obstacle distance.
class LikelihoodField2D
{
public:
  explicit LikelihoodField2D(LikelihoodFieldConfig config = {});

  // Recomputes obstacle distances from the current occupancy grid.
  void rebuild(const OccupancyGrid2D & grid);

  // Returns the nearest-obstacle distance for one grid cell, in meters.
  [[nodiscard]] float obstacleDistance(const GridIndex & index) const noexcept;
  [[nodiscard]] std::size_t cellCount() const noexcept;

private:
  [[nodiscard]] std::size_t linearIndex(const GridIndex & index) const noexcept;

  LikelihoodFieldConfig config_;
  std::size_t width_{0};
  // Stores the computed distance map in row-major order.
  std::vector<float> obstacle_distances_;
};

}  // namespace p03_slam_core
