#include "p03_slam_core/occupancy_grid_updater.hpp"

#include <cmath>

#include "p03_slam_core/grid_ray_tracing.hpp"
#include "p03_slam_core/point2d.hpp"

namespace p03_slam_core
{

// Stores the evidence values used for subsequent map updates.
OccupancyGridUpdater::OccupancyGridUpdater(InverseSensorModelConfig config)
: config_(config)
{
}

void OccupancyGridUpdater::addOccupiedEvidence(
  OccupancyGrid2D & grid, const GridIndex & endpoint) const
{
  const int radius_squared = config_.occupied_radius * config_.occupied_radius;
  for (int y = -config_.occupied_radius; y <= config_.occupied_radius; ++y) {
    for (int x = -config_.occupied_radius; x <= config_.occupied_radius; ++x) {
      const GridIndex neighbor{endpoint.x + x, endpoint.y + y};
      if (x * x + y * y <= radius_squared && grid.contains(neighbor)) {
        grid.addLogOdds(neighbor, config_.occupied_space_evidence);
      }
    }
  }
}

// Marks cells along one laser beam as free and optionally marks its endpoint as occupied.
void OccupancyGridUpdater::insertRay(
  OccupancyGrid2D & grid,
  const Point2D & sensor_origin_world,
  const Point2D & endpoint_world,
  const bool endpoint_is_occupied) const
{
  const auto start = grid.worldToGrid(sensor_origin_world);
  const auto end = grid.worldToGridUnbounded(endpoint_world);
  if (!start.has_value() || !end.has_value()) {
    return;
  }

  // Uses Bresenham's algorithm to find every cell crossed by the beam.
  const std::vector<GridIndex> cells = traceGridLine(*start, *end);
  for (std::size_t i = 0; i < cells.size(); ++i) {
    if (!grid.contains(cells[i])) {
      break;
    }

    const bool at_endpoint = i + 1U == cells.size();
    if (at_endpoint && endpoint_is_occupied) {
      addOccupiedEvidence(grid, cells[i]);
    } else {
      grid.addLogOdds(cells[i], config_.free_space_evidence);
    }
  }
}

// Converts each laser beam to map coordinates and inserts it into the grid.
void OccupancyGridUpdater::insertScan(
  OccupancyGrid2D & grid,
  const LaserScan2D & scan,
  const Pose2D & map_from_sensor) const
{
  const Point2D sensor_origin_world{map_from_sensor.x, map_from_sensor.y};

  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    const double measured_range = scan.ranges[i];
    if (!std::isfinite(measured_range) || measured_range < scan.range_min) {
      continue;
    }

    const bool endpoint_is_occupied = measured_range < scan.range_max;
    const double ray_length = endpoint_is_occupied ? measured_range : scan.range_max;
    const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;

    const Point2D endpoint_sensor{
      ray_length * std::cos(angle),
      ray_length * std::sin(angle)
    };
    const Point2D endpoint_world = transformPoint(map_from_sensor, endpoint_sensor);

    insertRay(grid, sensor_origin_world, endpoint_world, endpoint_is_occupied);
  }
}

}  // namespace p03_slam_core
