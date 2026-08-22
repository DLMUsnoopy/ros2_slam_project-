#include <gtest/gtest.h>

#include "p03_slam_core/occupancy_grid2d.hpp"

namespace p03_slam_core
{
namespace
{

OccupancyGrid2D makeTestGrid()
{
  OccupancyGridConfig config;
  config.width = 4;
  config.height = 3;
  config.resolution = 0.5;
  config.origin = {-1.0, -0.5};
  return OccupancyGrid2D(config);
}

TEST(OccupancyGrid2D, AllocatesConfiguredNumberOfCells)
{
  const OccupancyGrid2D grid = makeTestGrid();

  EXPECT_EQ(grid.cellCount(), 12U);
}

TEST(OccupancyGrid2D, ConvertsWorldPositionToGridIndex)
{
  const OccupancyGrid2D grid = makeTestGrid();

  const auto index = grid.worldToGrid(Point2D{0.2, 0.6});

  ASSERT_TRUE(index.has_value());
  EXPECT_EQ(index->x, 2);
  EXPECT_EQ(index->y, 2);
}

TEST(OccupancyGrid2D, RejectsPointsOutsideMapBounds)
{
  const OccupancyGrid2D grid = makeTestGrid();

  EXPECT_FALSE(grid.worldToGrid(Point2D{-1.01, 0.0}).has_value());
  EXPECT_FALSE(grid.worldToGrid(Point2D{1.0, 0.0}).has_value());
  EXPECT_FALSE(grid.worldToGrid(Point2D{0.0, 1.0}).has_value());
}

TEST(OccupancyGrid2D, StartsWithUnknownCells)
{
  const OccupancyGrid2D grid = makeTestGrid();
  const GridIndex index{1, 1};

  EXPECT_FLOAT_EQ(grid.logOdds(index), 0.0F);
  EXPECT_FALSE(grid.isObserved(index));
}

TEST(OccupancyGrid2D, AccumulatesEvidenceAndMarksCellObserved)
{
  OccupancyGrid2D grid = makeTestGrid();
  const GridIndex index{1, 1};

  grid.addLogOdds(index, 0.8F);
  grid.addLogOdds(index, -0.3F);

  EXPECT_FLOAT_EQ(grid.logOdds(index), 0.5F);
  EXPECT_TRUE(grid.isObserved(index));
}

TEST(OccupancyGrid2D, ClampsAccumulatedEvidence)
{
  OccupancyGrid2D grid = makeTestGrid();
  const GridIndex index{1, 1};

  grid.addLogOdds(index, 10.0F);
  EXPECT_FLOAT_EQ(grid.logOdds(index), grid.config().max_log_odds);

  grid.addLogOdds(index, -20.0F);
  EXPECT_FLOAT_EQ(grid.logOdds(index), grid.config().min_log_odds);
}

}  // namespace
}  // namespace p03_slam_core
