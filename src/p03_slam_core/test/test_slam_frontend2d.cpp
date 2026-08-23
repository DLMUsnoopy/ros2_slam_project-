#include <gtest/gtest.h>

#include "p03_slam_core/slam_frontend2d.hpp"

namespace p03_slam_core
{
namespace
{

TEST(SlamFrontend2D, InitializesMapFromFirstScan)
{
  SlamFrontendConfig config;
  config.occupancy_grid.width = 10;
  config.occupancy_grid.height = 10;
  config.occupancy_grid.resolution = 1.0;
  SlamFrontend2D frontend(config);

  LaserScan2D scan;
  scan.range_min = 0.1;
  scan.range_max = 10.0;
  scan.ranges = {2.0F};

  const SlamUpdate2D update = frontend.processScan(scan, {2.0, 3.0, 0.0});

  EXPECT_TRUE(frontend.isInitialized());
  EXPECT_FALSE(update.scan_matched);
  EXPECT_DOUBLE_EQ(update.map_from_lidar.x, 2.0);
  EXPECT_DOUBLE_EQ(update.map_from_lidar.y, 3.0);
  EXPECT_DOUBLE_EQ(update.map_from_odom.x, 0.0);
  EXPECT_TRUE(frontend.occupancyGrid().isObserved({4, 3}));
  EXPECT_GT(frontend.occupancyGrid().logOdds({4, 3}), 0.0F);
}

TEST(SlamFrontend2D, CorrectsOdometryDriftOnLaterScan)
{
  SlamFrontendConfig config;
  config.occupancy_grid.width = 10;
  config.occupancy_grid.height = 10;
  config.occupancy_grid.resolution = 1.0;
  config.matcher.standard_deviation = 0.5;
  config.matcher.linear_search_window = 1.0;
  config.matcher.angular_search_window = 0.0;
  config.matcher.linear_step = 1.0;
  SlamFrontend2D frontend(config);

  LaserScan2D scan;
  scan.range_min = 0.1;
  scan.range_max = 10.0;
  scan.ranges = {2.0F};

  static_cast<void>(frontend.processScan(scan, {2.0, 3.0, 0.0}));
  const SlamUpdate2D update = frontend.processScan(scan, {2.0, 4.0, 0.0});

  EXPECT_TRUE(update.scan_matched);
  EXPECT_DOUBLE_EQ(update.map_from_lidar.x, 2.0);
  EXPECT_DOUBLE_EQ(update.map_from_lidar.y, 3.0);
  EXPECT_DOUBLE_EQ(update.map_from_odom.x, 0.0);
  EXPECT_DOUBLE_EQ(update.map_from_odom.y, -1.0);
  EXPECT_DOUBLE_EQ(update.score, 1.0);
}

}  // namespace
}  // namespace p03_slam_core
