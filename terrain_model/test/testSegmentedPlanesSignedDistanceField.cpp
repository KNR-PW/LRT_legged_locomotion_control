#include <gtest/gtest.h>

#include <terrain_model/segmented_planes_model/SegmentedPlanesGridMapUtils.hpp>
#include <terrain_model/segmented_planes_model/SegmentedPlanesSignedDistanceField.hpp>

using namespace ocs2;
using namespace terrain_model;

namespace
{
  const scalar_t resolution = 0.04;
  const scalar_t heightMargin = 0.5;

  /** 2 x 2 m flat terrain at given height */
  grid_map::GridMap getFlatMap(float height)
  {
    grid_map::GridMap gridMap({elevationLayerName, groundLayerName});
    gridMap.setGeometry(grid_map::Length(2.0, 2.0), resolution, grid_map::Position(0.0, 0.0));
    gridMap.get(elevationLayerName).setConstant(height);
    gridMap.get(groundLayerName).setConstant(height);
    return gridMap;
  }
}

TEST(TestSegmentedPlanesSignedDistanceField, heightRange)
{
  // Distance has to be available up to heightMargin above the terrain, also for terrain below 0
  for(const float height : {-1.0f, -0.5f, 0.0f, 2.0f})
  {
    const auto gridMap = getFlatMap(height);
    const SegmentedPlanesSignedDistanceField sdf(gridMap, elevationLayerName,
      height - heightMargin, height + heightMargin);

    const scalar_t queryHeight = 0.4;
    EXPECT_NEAR(sdf.value({0.0, 0.0, height + queryHeight}), queryHeight, resolution);
    EXPECT_GT(sdf.derivative({0.0, 0.0, height + queryHeight}).z(), 0.0);
  }
}

TEST(TestSegmentedPlanesSignedDistanceField, noElevationData)
{
  auto gridMap = getFlatMap(0.3f);
  gridMap.get(elevationLayerName).setConstant(NAN);
  const SegmentedPlanesSignedDistanceField sdf(gridMap, elevationLayerName, NAN, NAN);

  // Flat ground at the height of the ground layer
  EXPECT_NEAR(sdf.value({0.0, 0.0, 0.5}), 0.2, 1e-6);
  EXPECT_TRUE(sdf.derivative({0.0, 0.0, 0.5}).isApprox(vector3_t::UnitZ()));
}

TEST(TestSegmentedPlanesSignedDistanceField, outsideMap)
{
  const auto gridMap = getFlatMap(0.3f);
  const SegmentedPlanesSignedDistanceField sdf(gridMap, elevationLayerName,
    0.3 - heightMargin, 0.3 + heightMargin);
  const std::unique_ptr<SignedDistanceField> clone(sdf.clone());

  const auto valueAndDerivative = clone->valueAndDerivative({5.0, -5.0, 0.1});
  EXPECT_NEAR(valueAndDerivative.first, -0.2, 1e-6);
  EXPECT_TRUE(valueAndDerivative.second.isApprox(vector3_t::UnitZ()));
}
