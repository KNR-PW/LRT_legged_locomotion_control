#include <gtest/gtest.h>

#include <grid_map_core/iterators/GridMapIterator.hpp>

#include <terrain_model/planar_model/PlanarFactoryFunctions.hpp>
#include <terrain_model/planar_model/PlanarTerrainModel.hpp>

using namespace ocs2;
using namespace terrain_model;

TEST(TestPlanarTerrainModel, gridMapMatchesPlane)
{
  // Tilted plane, so every cell has different height
  const TerrainPlane plane = computeTerrainPlane(vector3_t(1.0, 2.0, 0.5), vector3_t(0.0, 0.2, 0.3), 0.0);
  const PlanarTerrainModel terrainModel(plane);
  const auto& gridMap = terrainModel.getGridMapTerrain();

  for(grid_map::GridMapIterator iterator(gridMap); !iterator.isPastEnd(); ++iterator)
  {
    grid_map::Position cellPosition;
    gridMap.getPosition(*iterator, cellPosition);
    const scalar_t planeHeight = plane.projectPositionInWorldOntoPlaneAlongGravity(cellPosition).z();
    EXPECT_NEAR(gridMap.at(elevationLayerName, *iterator), planeHeight, 1e-5);
  }
}

TEST(TestPlanarTerrainModel, verticalPlane)
{
  // Surface normal along world X
  const matrix3_t orientationWorldToTerrain =
    TerrainPlane::getOrientationWorldToTerrainFromSurfaceNormalInWorld(vector3_t::UnitX());
  const TerrainPlane verticalPlane(vector3_t::Zero(), orientationWorldToTerrain);
  EXPECT_THROW(PlanarTerrainModel{verticalPlane}, std::invalid_argument);
}
