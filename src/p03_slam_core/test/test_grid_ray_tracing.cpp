#include <gtest/gtest.h>

#include "p03_slam_core/grid_ray_tracing.hpp"

namespace p03_slam_core
{
namespace
{

void expectCell(const GridIndex & cell, const int expected_x, const int expected_y)
{
  EXPECT_EQ(cell.x, expected_x);
  EXPECT_EQ(cell.y, expected_y);
}

TEST(GridRayTracing, TracesHorizontalLine)
{
  const auto cells = traceGridLine({1, 2}, {4, 2});

  ASSERT_EQ(cells.size(), 4U);
  expectCell(cells[0], 1, 2);
  expectCell(cells[1], 2, 2);
  expectCell(cells[2], 3, 2);
  expectCell(cells[3], 4, 2);
}

TEST(GridRayTracing, TracesDiagonalLine)
{
  const auto cells = traceGridLine({0, 0}, {3, 3});

  ASSERT_EQ(cells.size(), 4U);
  expectCell(cells[0], 0, 0);
  expectCell(cells[1], 1, 1);
  expectCell(cells[2], 2, 2);
  expectCell(cells[3], 3, 3);
}

TEST(GridRayTracing, TracesLineInReverseDirection)
{
  const auto cells = traceGridLine({3, 1}, {0, 1});

  ASSERT_EQ(cells.size(), 4U);
  expectCell(cells[0], 3, 1);
  expectCell(cells[1], 2, 1);
  expectCell(cells[2], 1, 1);
  expectCell(cells[3], 0, 1);
}

}  // namespace
}  // namespace p03_slam_core
