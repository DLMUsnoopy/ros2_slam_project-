#include <gtest/gtest.h>

#include <limits>

#include "p03_slam_core/occupancy_grid_updater.hpp"

namespace p03_slam_core
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

OccupancyGrid2D makeLineGrid()
{
  OccupancyGridConfig config;
  config.width = 5;
  config.height = 1;
  config.resolution = 1.0;
  return OccupancyGrid2D(config);
}

TEST(OccupancyGridUpdater, MarksRayAsFreeAndEndpointAsOccupied)
{
  OccupancyGrid2D grid = makeLineGrid();
  const OccupancyGridUpdater updater;

  updater.insertRay(grid, {0.5, 0.5}, {3.5, 0.5}, true);

  EXPECT_FLOAT_EQ(grid.logOdds({0, 0}), -0.4F);
  EXPECT_FLOAT_EQ(grid.logOdds({1, 0}), -0.4F);
  EXPECT_FLOAT_EQ(grid.logOdds({2, 0}), -0.4F);
  EXPECT_FLOAT_EQ(grid.logOdds({3, 0}), 0.85F);
}

TEST(OccupancyGridUpdater, MarksMaximumRangeEndpointAsFree)
{
  OccupancyGrid2D grid = makeLineGrid();
  const OccupancyGridUpdater updater;

  updater.insertRay(grid, {0.5, 0.5}, {3.5, 0.5}, false);

  EXPECT_FLOAT_EQ(grid.logOdds({0, 0}), -0.4F);
  EXPECT_FLOAT_EQ(grid.logOdds({1, 0}), -0.4F);
  EXPECT_FLOAT_EQ(grid.logOdds({2, 0}), -0.4F);
  EXPECT_FLOAT_EQ(grid.logOdds({3, 0}), -0.4F);
}

TEST(OccupancyGridUpdater, UpdatesFreeSpaceUntilRayLeavesMap)
{
  OccupancyGrid2D grid = makeLineGrid();
  const OccupancyGridUpdater updater;

  updater.insertRay(grid, {1.5, 0.5}, {8.5, 0.5}, true);

  EXPECT_FLOAT_EQ(grid.logOdds({1, 0}), -0.4F);
  EXPECT_FLOAT_EQ(grid.logOdds({2, 0}), -0.4F);
  EXPECT_FLOAT_EQ(grid.logOdds({3, 0}), -0.4F);
  EXPECT_FLOAT_EQ(grid.logOdds({4, 0}), -0.4F);
}

TEST(OccupancyGridUpdater, InflatesOccupiedEndpointByConfiguredRadius)
{
  OccupancyGrid2D grid = makeLineGrid();
  InverseSensorModelConfig config;
  config.occupied_radius = 1;
  const OccupancyGridUpdater updater(config);

  updater.insertRay(grid, {0.5, 0.5}, {3.5, 0.5}, true);

  EXPECT_FLOAT_EQ(grid.logOdds({2, 0}), 0.45F);
  EXPECT_FLOAT_EQ(grid.logOdds({3, 0}), 0.85F);
  EXPECT_FLOAT_EQ(grid.logOdds({4, 0}), 0.85F);
}

TEST(OccupancyGridUpdater, InsertsScanUsingSensorPose)
{
  OccupancyGridConfig grid_config;
  grid_config.width = 6;
  grid_config.height = 6;
  grid_config.resolution = 1.0;
  OccupancyGrid2D grid(grid_config);
  const OccupancyGridUpdater updater;

  LaserScan2D scan;
  scan.angle_min = 0.0;
  scan.angle_increment = 0.5 * kPi;
  scan.range_max = 3.0;
  scan.ranges = {2.0F, 1.0F};

  const Pose2D map_from_sensor{2.5, 2.5, 0.5 * kPi};
  updater.insertScan(grid, scan, map_from_sensor);

  EXPECT_FLOAT_EQ(grid.logOdds({2, 2}), -0.8F);
  EXPECT_FLOAT_EQ(grid.logOdds({2, 3}), -0.4F);
  EXPECT_FLOAT_EQ(grid.logOdds({2, 4}), 0.85F);
  EXPECT_FLOAT_EQ(grid.logOdds({1, 2}), 0.85F);
}

TEST(OccupancyGridUpdater, IgnoresInvalidAndTooCloseRanges)
{
  OccupancyGrid2D grid = makeLineGrid();
  const OccupancyGridUpdater updater;

  LaserScan2D scan;
  scan.range_min = 0.25;
  scan.range_max = 3.0;
  scan.ranges = {
    -std::numeric_limits<float>::infinity(),
    0.1F,
    2.0F
  };

  updater.insertScan(grid, scan, {0.5, 0.5, 0.0});

  EXPECT_FLOAT_EQ(grid.logOdds({0, 0}), -0.4F);
  EXPECT_FLOAT_EQ(grid.logOdds({1, 0}), -0.4F);
  EXPECT_FLOAT_EQ(grid.logOdds({2, 0}), 0.85F);
}

}  // namespace
}  // namespace p03_slam_core
