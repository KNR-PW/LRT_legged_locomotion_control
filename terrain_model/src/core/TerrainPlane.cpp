//
// Created by rgrandia on 21.04.20.
// Modified by Bartłomiej Krajewski (https://github.com/BartlomiejK2) on 29.08.2025 
//

#include <terrain_model/core/TerrainPlane.hpp>

namespace terrain_model 
{
  using namespace ocs2;
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  TerrainPlane::TerrainPlane()
    : positionInWorld_(vector3_t::Zero()), orientationWorldToTerrain_(matrix3_t::Identity()) {}
   
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  TerrainPlane::TerrainPlane(vector3_t positionInWorld, matrix3_t orientationWorldToTerrain)
    : positionInWorld_(std::move(positionInWorld)),
      orientationWorldToTerrain_(std::move(orientationWorldToTerrain)) {}

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  const vector3_t& TerrainPlane::getPosition() const
  {
    return positionInWorld_;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
	const matrix3_t& TerrainPlane::getOrientationToTerrain() const
  {
    return orientationWorldToTerrain_;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  vector3_t TerrainPlane::getSurfaceNormalInWorld() const
  {
    return orientationWorldToTerrain_.row(2).transpose();
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  vector3_t TerrainPlane::getPositionInTerrainFrameFromPositionInWorld(
    const vector3_t& positionWorld) const
  {
    return orientationWorldToTerrain_ * (positionWorld - positionInWorld_);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  vector3_t TerrainPlane::getPositionInWorldFrameFromPositionInTerrain(
    const vector3_t& positionInTerrain) const
  {
    return orientationWorldToTerrain_.transpose() * positionInTerrain + positionInWorld_;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  scalar_t TerrainPlane::getTerrainSignedDistanceFromPositionInWorld(
    const vector3_t& positionWorld) const
  {
    const vector3_t surfaceNormal = getSurfaceNormalInWorld();
    return surfaceNormal.dot(positionWorld - positionInWorld_);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  vector3_t TerrainPlane::projectPositionInWorldOntoPlaneAlongGravity(
    const vector2_t& positionXYWorld) const
  {
    // solve surfaceNormal.dot(projectedPosition - terrainPlane.positionInWorld) = 0
    // Distance = positionWorld.z() - projectedPosition.z()
    const vector3_t surfaceNormal = getSurfaceNormalInWorld();
    const scalar_t projectedPositionZ = (surfaceNormal.dot(positionInWorld_)
      - positionXYWorld.x() * surfaceNormal.x() - positionXYWorld.y()
      * surfaceNormal.y()) / surfaceNormal.z();
    return {positionXYWorld.x(), positionXYWorld.y(), projectedPositionZ};
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  vector3_t TerrainPlane::projectPositionInWorldOntoPlane(
    const vector3_t& positionWorld) const
  {
    const vector3_t surfaceNormal = getSurfaceNormalInWorld();
    return surfaceNormal.dot(positionInWorld_ - positionWorld) * surfaceNormal + positionWorld;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  vector3_t TerrainPlane::projectPositionInWorldOntoPlaneAlongGravity(
    const vector3_t& positionWorld) const
  {
    return projectPositionInWorldOntoPlaneAlongGravity(
      vector2_t{positionWorld.x(), positionWorld.y()});
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  vector3_t TerrainPlane::projectVectorInWorldOntoPlaneAlongGravity(
    const vector3_t& vectorInWorld) const
  {
    const vector3_t surfaceNormal = getSurfaceNormalInWorld();
    vector3_t projectedVector = vectorInWorld;
    // solve
    // 1. projectedVector.x() = vectorInWorld.x();
    // 2. projectedVector.y() = vectorInWorld.y();
    // 3. surfaceNormal.dot(projectedVector) = 0
    projectedVector.z() = -(vectorInWorld.x() * surfaceNormal.x() + vectorInWorld.y()
      * surfaceNormal.y())
      / surfaceNormal.z();
    return projectedVector;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void TerrainPlane::changeHeadingVector(const vector3_t headingVectorInWorld)
  {
    const vector3_t normalPlane = getSurfaceNormalInWorld();
    const vector3_t projectedHeading = projectVectorInWorldOntoPlaneAlongGravity(headingVectorInWorld);

    // Vertical or zero heading has no direction on the plane, keep current orientation
    const scalar_t minimalHeadingNorm = 1e-9;
    if (!(projectedHeading.norm() > minimalHeadingNorm))
    {
      return;
    }

    const vector3_t newXAxis = projectedHeading.normalized();
    const vector3_t newYAxis = normalPlane.cross(newXAxis);

    orientationWorldToTerrain_.row(0) = newXAxis.transpose();
    orientationWorldToTerrain_.row(1) = newYAxis.transpose();
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  Eigen::Matrix<scalar_t, 2, 3> TerrainPlane::getTangentialBasisFromSurfaceNormal(
    const vector3_t& surfaceNormal)
  {
    // Cross with any vector that is not equal to surfaceNormal
    Eigen::Vector3d yAxisInGlobal = surfaceNormal.cross(Eigen::Vector3d::UnitX());

    // Normalize the yAxis. Need to pick a different direction if z happened to intersect with unitX
    const auto ySquaredNorm = yAxisInGlobal.squaredNorm();
    const double crossTolerance = 1e-3;
    if (ySquaredNorm > crossTolerance)
    {
        yAxisInGlobal /= std::sqrt(ySquaredNorm);
    }
    else
    {
        // normal was almost equal to unitX. Pick the y-axis in a different way (approximately equal to unitY):
        yAxisInGlobal = surfaceNormal.cross(Eigen::Vector3d::UnitY().cross(surfaceNormal)).normalized();
    }
    // Assumes the surface normal is normalized
    Eigen::Matrix<scalar_t, 2, 3> tangentBasis;
    tangentBasis.row(0) = yAxisInGlobal.cross(surfaceNormal);
    tangentBasis.row(1) = yAxisInGlobal;
    return tangentBasis;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  matrix3_t TerrainPlane::getOrientationWorldToTerrainFromSurfaceNormalInWorld(
    const vector3_t& surfaceNormal)
  {
      Eigen::Matrix<scalar_t, 2, 3> tangents = getTangentialBasisFromSurfaceNormal(surfaceNormal);
      matrix3_t orientationWorldToTerrain;
      orientationWorldToTerrain.topRows(2) = tangents;
      orientationWorldToTerrain.row(2) = surfaceNormal.transpose();
      return orientationWorldToTerrain;
  }
} // namespace terrain_model
