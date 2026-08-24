#include <gtest/gtest.h>

#include "p03_slam_core/likelihood_field_matcher2d.hpp"

namespace p03_slam_core
{
namespace
{

TEST(LikelihoodFieldMatcher2D, GivesHigherScoreToAlignedScan)
{
  OccupancyGridConfig grid_config;
  grid_config.width = 8;
  grid_config.height = 6;
  grid_config.resolution = 1.0;
  OccupancyGrid2D grid(grid_config);
  grid.addLogOdds({5, 2}, 1.0F);

  LikelihoodField2D field;
  field.rebuild(grid);

  LaserScan2D scan;
  scan.range_min = 0.1;
  scan.range_max = 10.0;
  scan.ranges = {2.0F};

  LikelihoodFieldMatcherConfig matcher_config;
  matcher_config.standard_deviation = 0.5;
  LikelihoodFieldMatcher2D matcher(matcher_config);

  const double aligned_score = matcher.scorePose(grid, field, scan, {3.0, 2.0, 0.0});
  const double displaced_score = matcher.scorePose(grid, field, scan, {3.0, 3.0, 0.0});

  EXPECT_DOUBLE_EQ(aligned_score, 1.0);
  EXPECT_GT(aligned_score, displaced_score);
}

TEST(LikelihoodFieldMatcher2D, FindsAlignedPoseNearInitialGuess)
{
  OccupancyGridConfig grid_config;
  grid_config.width = 8;
  grid_config.height = 6;
  grid_config.resolution = 1.0;
  OccupancyGrid2D grid(grid_config);
  grid.addLogOdds({5, 2}, 1.0F);

  LikelihoodField2D field;
  field.rebuild(grid);

  LaserScan2D scan;
  scan.range_min = 0.1;
  scan.range_max = 10.0;
  scan.ranges = {2.0F};

  LikelihoodFieldMatcherConfig matcher_config;
  matcher_config.linear_search_window = 1.0;
  matcher_config.angular_search_window = 0.0;
  matcher_config.linear_step = 1.0;
  LikelihoodFieldMatcher2D matcher(matcher_config);

  const ScanMatchResult2D result = matcher.match(grid, field, scan, {3.0, 3.0, 0.0});

  EXPECT_DOUBLE_EQ(result.map_from_lidar.x, 3.0);
  EXPECT_DOUBLE_EQ(result.map_from_lidar.y, 2.0);
  EXPECT_DOUBLE_EQ(result.map_from_lidar.yaw, 0.0);
  EXPECT_DOUBLE_EQ(result.score, 1.0);
  EXPECT_GT(result.score, result.prior_score);
  EXPECT_EQ(result.matched_point_count, 1U);
  EXPECT_TRUE(result.accepted);
}

TEST(LikelihoodFieldMatcher2D, RejectsCorrectionWithoutScoreImprovement)
{
  OccupancyGridConfig grid_config;
  grid_config.width = 8;
  grid_config.height = 6;
  grid_config.resolution = 1.0;
  OccupancyGrid2D grid(grid_config);
  grid.addLogOdds({5, 2}, 1.0F);

  LikelihoodField2D field;
  field.rebuild(grid);

  LaserScan2D scan;
  scan.range_min = 0.1;
  scan.range_max = 10.0;
  scan.ranges = {2.0F};

  LikelihoodFieldMatcherConfig matcher_config;
  matcher_config.min_score_improvement = 0.01;
  LikelihoodFieldMatcher2D matcher(matcher_config);

  const ScanMatchResult2D result = matcher.match(grid, field, scan, {3.0, 2.0, 0.0});

  EXPECT_DOUBLE_EQ(result.score, 1.0);
  EXPECT_DOUBLE_EQ(result.prior_score, 1.0);
  EXPECT_FALSE(result.accepted);
}

}  // namespace
}  // namespace p03_slam_core
