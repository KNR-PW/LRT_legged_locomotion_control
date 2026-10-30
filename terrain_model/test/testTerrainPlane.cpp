//
// Created by rgrandia on 29.04.20.
//

#include <gtest/gtest.h>

#include <terrain_model/core/TerrainPlane.hpp>
#include "Rotations.hpp"

using namespace ocs2;
using namespace terrain_model;

TerrainPlane getRandomTerrain() {
  vector3_t eulerXYZ = vector3_t::Random();
  return TerrainPlane(vector3_t::Random(), rotationMatrixBaseToOrigin(eulerXYZ));
}

TEST(TestTerrainPlane, surfaceNormal) {
  TerrainPlane canonicalPlane(vector3_t::Zero(), matrix3_t::Identity());

  ASSERT_TRUE(canonicalPlane.getSurfaceNormalInWorld().isApprox(vector3_t{0.0, 0.0, 1.0}));

  vector3_t eulerXYZ{0.0, 0.0, 0.3};
  const auto yawRotation = rotationMatrixBaseToOrigin(eulerXYZ);

  TerrainPlane yawRotatedPlane(vector3_t::Zero(), yawRotation * matrix3_t::Identity());

  ASSERT_TRUE(yawRotatedPlane.getSurfaceNormalInWorld().isApprox(vector3_t{0.0, 0.0, 1.0}));
}

TEST(TestTerrainPlane, tangentialBasisFromSurfaceNormal) {
  const auto randomPlane = getRandomTerrain();
  const auto surfaceNormal = randomPlane.getSurfaceNormalInWorld();

  const auto tangentialBasis = TerrainPlane::getTangentialBasisFromSurfaceNormal(surfaceNormal);

  // Normalized
  ASSERT_DOUBLE_EQ(tangentialBasis.row(0).norm(), 1.0);
  ASSERT_DOUBLE_EQ(tangentialBasis.row(1).norm(), 1.0);

  // Orthogonal
  const double tol = 1e-9;
  ASSERT_LT(std::abs(tangentialBasis.row(0).dot(surfaceNormal)), tol);
  ASSERT_LT(std::abs(tangentialBasis.row(1).dot(surfaceNormal)), tol);
  ASSERT_LT(std::abs(tangentialBasis.row(0).dot(tangentialBasis.row(1))), tol);
}

TEST(TestTerrainPlane, tangentialBasisFromSurfaceNormalUnitX) {
  const auto surfaceNormal = vector3_t::UnitX();
  const auto tangentialBasis = TerrainPlane::getTangentialBasisFromSurfaceNormal(surfaceNormal);

  // Normalized
  ASSERT_DOUBLE_EQ(tangentialBasis.row(0).norm(), 1.0);
  ASSERT_DOUBLE_EQ(tangentialBasis.row(1).norm(), 1.0);

  // Orthogonal
  const double tol = 1e-9;
  ASSERT_LT(std::abs(tangentialBasis.row(0).dot(surfaceNormal)), tol);
  ASSERT_LT(std::abs(tangentialBasis.row(1).dot(surfaceNormal)), tol);
  ASSERT_LT(std::abs(tangentialBasis.row(0).dot(tangentialBasis.row(1))), tol);

  // Y is still unit y
  ASSERT_TRUE(tangentialBasis.row(1).transpose().isApprox(vector3_t::UnitY()));
}

TEST(TestTerrainPlane, changeHeadingVector) {
  auto randomPlane = getRandomTerrain();
  const vector3_t surfaceNormal = randomPlane.getSurfaceNormalInWorld();
  const vector3_t newHeadingVector = vector3_t::UnitX();
  const vector3_t projectedHeading = randomPlane.projectVectorInWorldOntoPlaneAlongGravity(newHeadingVector).normalized();
  randomPlane.changeHeadingVector(newHeadingVector);

  const auto newRotation = randomPlane.getOrientationToTerrain().transpose();

  const double tol = 1e-9;
  ASSERT_TRUE(newRotation.col(0).isApprox(projectedHeading, tol));
  ASSERT_TRUE(newRotation.col(1).isApprox(surfaceNormal.cross(projectedHeading), tol));
  ASSERT_TRUE(newRotation.col(2).isApprox(surfaceNormal, tol));
}

TEST(TestTerrainPlane, changeHeadingVectorVertical) {
  // Vertical heading has no direction on horizontal plane, orientation should not change
  vector3_t eulerXYZ{0.0, 0.0, 0.3};
  TerrainPlane plane(vector3_t::Zero(), rotationMatrixOriginToBase(eulerXYZ));
  const matrix3_t orientationBefore = plane.getOrientationToTerrain();

  plane.changeHeadingVector(vector3_t::UnitZ());
  ASSERT_TRUE(plane.getOrientationToTerrain().isApprox(orientationBefore));

  plane.changeHeadingVector(vector3_t::Zero());
  ASSERT_TRUE(plane.getOrientationToTerrain().isApprox(orientationBefore));
}

TEST(TestTerrainPlane, orientationWorldToTerrainFromSurfaceNormalInWorld) {
  const vector3_t surfaceNormal = vector3_t::Random().normalized();

  // Extract
  const matrix3_t R_WtoT = TerrainPlane::getOrientationWorldToTerrainFromSurfaceNormalInWorld(surfaceNormal);
  const vector3_t xAxisInWorld = R_WtoT.row(0).transpose();
  const vector3_t yAxisInWorld = R_WtoT.row(1).transpose();
  const vector3_t zAxisInWorld = R_WtoT.row(2).transpose();

  const double tol = 1e-9;
  ASSERT_LT(std::abs(R_WtoT.determinant() - 1.0), tol);

  // z-axis is the surface normal
  ASSERT_TRUE(zAxisInWorld.isApprox(surfaceNormal));

  // Right hand coordinate system
  ASSERT_TRUE(zAxisInWorld.cross(xAxisInWorld).isApprox(yAxisInWorld));

  // Ortogonal
  ASSERT_LT(std::abs(xAxisInWorld.dot(yAxisInWorld)), tol);
  ASSERT_LT(std::abs(xAxisInWorld.dot(zAxisInWorld)), tol);
  ASSERT_LT(std::abs(yAxisInWorld.dot(zAxisInWorld)), tol);
}

TEST(TestTerrainPlane, orientationWorldToTerrainFromSurfaceNormalInWorld_identity) {
  // If the normal is in z direction in world. We want to retrieve the identity rotation. (local and world frame are aligned)
  const matrix3_t R_WtoT = TerrainPlane::getOrientationWorldToTerrainFromSurfaceNormalInWorld(vector3_t::UnitZ());
  ASSERT_TRUE(R_WtoT.isApprox(matrix3_t::Identity()));
}

TEST(TestTerrainPlane, projectPositionInWorldOntoPlane) {
  const auto randomPlane = getRandomTerrain();

  // Origin of plane is projected to itself
  ASSERT_TRUE(randomPlane.projectPositionInWorldOntoPlane(randomPlane.getPosition()).isApprox(randomPlane.getPosition()));

  // Random point projected
  const auto projectedPosition = randomPlane.projectPositionInWorldOntoPlane(vector3_t::Random());

  // Distance to plane
  const double tol = 1e-9;
  ASSERT_LT(std::abs(randomPlane.getTerrainSignedDistanceFromPositionInWorld(projectedPosition)), tol);

  // Double projection
  ASSERT_TRUE(randomPlane.projectPositionInWorldOntoPlane(projectedPosition).isApprox(projectedPosition));
}

TEST(TestTerrainPlane, projectPositionInWorldOntoPlaneAlongGravity_flatTerrain) {
  vector3_t eulerXYZ(0, 0, 0.3);
  TerrainPlane flatTerrain = {vector3_t::Random(), rotationMatrixOriginToBase(eulerXYZ)};

  // Origin of plane is projected to itself
  ASSERT_TRUE(flatTerrain.projectPositionInWorldOntoPlaneAlongGravity(flatTerrain.getPosition()).isApprox(flatTerrain.getPosition()));

  // Any point in the horizontal plane should stay  point projected
  const vector3_t queryPosition = {1.0, 1.0, 0.0};
  const auto projectedPosition = flatTerrain.projectPositionInWorldOntoPlaneAlongGravity(queryPosition);

  ASSERT_TRUE(queryPosition.head(2).isApprox(projectedPosition.head(2)));
  ASSERT_DOUBLE_EQ(projectedPosition.z(), flatTerrain.getPosition().z());
}

TEST(TestTerrainPlane, projectPositionInWorldOntoPlaneAlongGravity_randomTerrain) {
  const auto randomPlane = getRandomTerrain();

  // Origin of plane is projected to itself
  ASSERT_TRUE(randomPlane.projectPositionInWorldOntoPlaneAlongGravity(randomPlane.getPosition()).isApprox(randomPlane.getPosition()));

  // Random point projected
  const vector3_t queryPosition = vector3_t::Random();
  const auto projectedPosition = randomPlane.projectPositionInWorldOntoPlaneAlongGravity(queryPosition);

  // x, y position remained to same
  ASSERT_DOUBLE_EQ(projectedPosition.x(), queryPosition.x());
  ASSERT_DOUBLE_EQ(projectedPosition.y(), queryPosition.y());

  // Distance to plane
  const double tol = 1e-9;
  ASSERT_LT(std::abs(randomPlane.getTerrainSignedDistanceFromPositionInWorld(projectedPosition)), tol);

  // Double projection
  ASSERT_TRUE(randomPlane.projectPositionInWorldOntoPlane(projectedPosition).isApprox(projectedPosition));
}