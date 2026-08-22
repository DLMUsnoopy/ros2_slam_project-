#pragma once

#include <vector>

#include "p03_slam_core/occupancy_grid2d.hpp"

namespace p03_slam_core
{

// Returns the ordered grid cells crossed by a line, including both endpoints.
[[nodiscard]] std::vector<GridIndex> traceGridLine(
  const GridIndex & start,
  const GridIndex & end);

}  // namespace p03_slam_core
