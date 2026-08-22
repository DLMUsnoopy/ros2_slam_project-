#pragma once

#include "p03_slam_core/laser_scan2d.hpp"
#include "p03_slam_core/occupancy_grid2d.hpp"
#include "p03_slam_core/pose2d.hpp"

namespace p03_slam_core
{

// Defines how strongly one laser beam changes free and occupied cells.
struct InverseSensorModelConfig
{
  float free_space_evidence{-0.4F};
  float occupied_space_evidence{0.85F};
};

// Converts inverse sensor model results into occupancy-grid updates.
class OccupancyGridUpdater
{
public:
  explicit OccupancyGridUpdater(InverseSensorModelConfig config = {});

  // Inserts one laser beam into an occupancy grid.
  void insertRay(
    OccupancyGrid2D & grid,
    const Point2D & sensor_origin_world,
    const Point2D & endpoint_world,
    bool endpoint_is_occupied) const;

  // Inserts every beam from one laser scan into an occupancy grid.
  void insertScan(
    OccupancyGrid2D & grid,
    const LaserScan2D & scan,
    const Pose2D & map_from_sensor) const;

private:
  InverseSensorModelConfig config_;
};

}  // namespace p03_slam_core
