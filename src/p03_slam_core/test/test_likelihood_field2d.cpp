#include <gtest/gtest.h>

#include "p03_slam_core/likelihood_field2d.hpp"

namespace p03_slam_core
{
namespace
{

TEST(LikelihoodField2D, ComputesAndLimitsObstacleDistances)
{
  OccupancyGridConfig grid_config;
  grid_config.width = 5;
  grid_config.height = 5;
  grid_config.resolution = 0.5;
  OccupancyGrid2D grid(grid_config);
  grid.addLogOdds({2, 2}, 1.0F);

  LikelihoodFieldConfig field_config;
  field_config.max_obstacle_distance = 0.9F;
  LikelihoodField2D field(field_config);
  field.rebuild(grid);

  EXPECT_EQ(field.cellCount(), 25U);
  EXPECT_FLOAT_EQ(field.obstacleDistance({2, 2}), 0.0F);
  EXPECT_FLOAT_EQ(field.obstacleDistance({3, 2}), 0.5F);
  EXPECT_NEAR(field.obstacleDistance({3, 3}), 0.5F * 1.41421356237F, 1e-6F);
  EXPECT_FLOAT_EQ(field.obstacleDistance({0, 2}), 0.9F);
}

}  // namespace
}  // namespace p03_slam_core
