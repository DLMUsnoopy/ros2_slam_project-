#include "p03_slam_core/grid_ray_tracing.hpp"

#include <cstdlib>

namespace p03_slam_core
{

// Traces an integer grid line using Bresenham's algorithm.
std::vector<GridIndex> traceGridLine(const GridIndex & start, const GridIndex & end)
{
  std::vector<GridIndex> cells;

  int x = start.x;
  int y = start.y;
  const int delta_x = std::abs(end.x - start.x);
  const int delta_y = -std::abs(end.y - start.y);
  // Selects whether each x update moves right or left.
  const int step_x = start.x < end.x ? 1 : -1;
  const int step_y = start.y < end.y ? 1 : -1;
  int error = delta_x + delta_y;

  while (true) {
    cells.push_back({x, y});

    if (x == end.x && y == end.y) {
      break;
    }

    const int doubled_error = 2 * error;
    if (doubled_error >= delta_y) {
      error += delta_y;
      x += step_x;
    }
    if (doubled_error <= delta_x) {
      error += delta_x;
      y += step_y;
    }
  }

  return cells;
}

}  // namespace p03_slam_core
