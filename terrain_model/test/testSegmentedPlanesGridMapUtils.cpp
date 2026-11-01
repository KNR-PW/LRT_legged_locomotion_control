#include <gtest/gtest.h>

#include <grid_map_core/iterators/GridMapIterator.hpp>

#include <terrain_model/segmented_planes_model/SegmentedPlanesGridMapUtils.hpp>

using namespace ocs2;
using namespace terrain_model;

namespace
{
  /** 2 x 2 m map centered at (1, -1), ground height is equal to x so map edges are distinguishable */
  grid_map::GridMap getGroundMap()
  {
    grid_map::GridMap gridMap({groundLayerName});
    gridMap.setGeometry(grid_map::Length(2.0, 2.0), 0.1, grid_map::Position(1.0, -1.0));
    for(grid_map::GridMapIterator iterator(gridMap); !iterator.isPastEnd(); ++iterator)
    {
      grid_map::Position position;
      gridMap.getPosition(*iterator, position);
      gridMap.at(groundLayerName, *iterator) = position.x();
    }
    return gridMap;
  }
}

TEST(TestSegmentedPlanesGridMapUtils, groundHeightInsideMap)
{
  const auto gridMap = getGroundMap();
  EXPECT_NEAR(getGroundHeightAtClosestCell(gridMap, vector2_t(1.05, -1.0)), 1.05, 1e-6);
}

TEST(TestSegmentedPlanesGridMapUtils, groundHeightOutsideMap)
{
  const auto gridMap = getGroundMap();
  // Map spans x in [0, 2], edge cells have centers at x = 0.05 and x = 1.95
  EXPECT_NEAR(getGroundHeightAtClosestCell(gridMap, vector2_t(10.0, -1.0)), 1.95, 1e-6);
  EXPECT_NEAR(getGroundHeightAtClosestCell(gridMap, vector2_t(-1e6, 5.0)), 0.05, 1e-6);
  EXPECT_NEAR(getGroundHeightAtClosestCell(gridMap, vector2_t(2.0 + 1e-9, -1.0)), 1.95, 1e-6);
}

TEST(TestSegmentedPlanesGridMapUtils, groundHeightNotFinite)
{
  auto gridMap = getGroundMap();
  gridMap.get(groundLayerName).setConstant(NAN);
  EXPECT_EQ(getGroundHeightAtClosestCell(gridMap, vector2_t(10.0, -1.0)), 0.0);
}

TEST(TestSegmentedPlanesGridMapUtils, clipLineInsideMap)
{
  const auto gridMap = getGroundMap();
  const vector2_t start(0.5, -1.5);
  const vector2_t end(1.5, -0.5);
  vector2_t clippedStart, clippedEnd;
  ASSERT_TRUE(clipLineToGridMap(gridMap, start, end, clippedStart, clippedEnd));
  EXPECT_TRUE(clippedStart.isApprox(start));
  EXPECT_TRUE(clippedEnd.isApprox(end));
}

TEST(TestSegmentedPlanesGridMapUtils, clipLineCrossingMapEdge)
{
  const auto gridMap = getGroundMap();
  vector2_t clippedStart, clippedEnd;
  ASSERT_TRUE(clipLineToGridMap(gridMap, vector2_t(1.0, -1.0), vector2_t(5.0, -1.0),
    clippedStart, clippedEnd));
  EXPECT_TRUE(clippedStart.isApprox(vector2_t(1.0, -1.0)));
  // Half of the cell inside of the map edge at x = 2
  EXPECT_NEAR(clippedEnd.x(), 1.95, 1e-9);
  EXPECT_NEAR(clippedEnd.y(), -1.0, 1e-9);
  EXPECT_TRUE(gridMap.isInside(clippedEnd));
}

TEST(TestSegmentedPlanesGridMapUtils, clipLineOutsideMap)
{
  const auto gridMap = getGroundMap();
  vector2_t clippedStart, clippedEnd;
  // Parallel to the map edge
  EXPECT_FALSE(clipLineToGridMap(gridMap, vector2_t(-0.5, -1.5), vector2_t(-0.5, -0.5),
    clippedStart, clippedEnd));
  // Diagonal passing next to the map corner
  EXPECT_FALSE(clipLineToGridMap(gridMap, vector2_t(1.5, 0.5), vector2_t(2.5, -0.5),
    clippedStart, clippedEnd));
  // Not finite
  EXPECT_FALSE(clipLineToGridMap(gridMap, vector2_t(NAN, 0.0), vector2_t(1.0, -1.0),
    clippedStart, clippedEnd));
}
