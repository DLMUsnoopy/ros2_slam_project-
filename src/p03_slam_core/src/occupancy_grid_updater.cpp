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

// Marks cells along one laser beam as free and optionally marks its endpoint as occupied.
void OccupancyGridUpdater::insertRay(
  OccupancyGrid2D & grid,
  const Point2D & sensor_origin_world,
  const Point2D & endpoint_world,
  const bool endpoint_is_occupied) const
{
  const auto start = grid.worldToGrid(sensor_origin_world);
  const auto end = grid.worldToGrid(endpoint_world);
  if (!start.has_value() || !end.has_value()) {
    return;
  }

  // Uses Bresenham's algorithm to find every cell crossed by the beam.
  const std::vector<GridIndex> cells = traceGridLine(*start, *end);
  for (std::size_t i = 0; i + 1 < cells.size(); ++i) {
    grid.addLogOdds(cells[i], config_.free_space_evidence);
  }

  const float endpoint_evidence = endpoint_is_occupied ?
    config_.occupied_space_evidence : config_.free_space_evidence;
  grid.addLogOdds(cells.back(), endpoint_evidence);
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
