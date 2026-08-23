#include <gtest/gtest.h>

#include "p03_slam_ros/occupancy_grid_conversion.hpp"

namespace p03_slam_ros
{
namespace
{

TEST(OccupancyGridConversion, ConvertsMetadataAndCellValues)
{
  p03_slam_core::OccupancyGridConfig config;
  config.width = 3;
  config.height = 1;
  config.resolution = 0.5;
  config.origin = {-1.0, -2.0};
  p03_slam_core::OccupancyGrid2D grid(config);
  grid.addLogOdds({1, 0}, -4.0F);
  grid.addLogOdds({2, 0}, 4.0F);

  const nav_msgs::msg::OccupancyGrid message = toRosOccupancyGrid(grid);

  EXPECT_EQ(message.info.width, 3U);
  EXPECT_EQ(message.info.height, 1U);
  EXPECT_FLOAT_EQ(message.info.resolution, 0.5F);
  EXPECT_DOUBLE_EQ(message.info.origin.position.x, -1.0);
  EXPECT_DOUBLE_EQ(message.info.origin.position.y, -2.0);
  EXPECT_DOUBLE_EQ(message.info.origin.orientation.w, 1.0);
  ASSERT_EQ(message.data.size(), 3U);
  EXPECT_EQ(message.data[0], -1);
  EXPECT_EQ(message.data[1], 2);
  EXPECT_EQ(message.data[2], 98);
}

}  // namespace
}  // namespace p03_slam_ros
