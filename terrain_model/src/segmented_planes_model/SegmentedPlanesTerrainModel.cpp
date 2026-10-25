//
// Created by rgrandia on 23.06.20.
// Modified by Bartłomiej Krajewski (https://github.com/BartlomiejK2) on 30.04.2026
//

#include <terrain_model/segmented_planes_model/SegmentedPlanesTerrainModel.hpp>
#include <terrain_model/segmented_planes_model/SegmentedPlanesGridMapUtils.hpp>

#include <algorithm>
#include <cmath>
#include <sstream>

#include <grid_map_filters_rsl/lookup.hpp>

#include <convex_plane_decomposition/ConvexRegionGrowing.h>
#include <convex_plane_decomposition/SegmentedPlaneProjection.h>

namespace terrain_model
{
  using namespace ocs2;
  using namespace grid_map;
  using namespace convex_plane_decomposition;
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  SegmentedPlanesTerrainModel::SegmentedPlanesTerrainModel(
    const PlanarTerrain& planarTerrain, std::string referenceFrameName): 
      TerrainModel(std::move(referenceFrameName)),
      planarTerrain_(planarTerrain), 
      signedDistanceField_(nullptr),
      elevationData_(&planarTerrain_.gridMap.get(elevationLayerName))
  {
    planarTerrain_.gridMap.setFrameId(referenceFrameName_);
    const auto ranges = getSignedDistanceRange(planarTerrain_.gridMap, elevationLayerName);
    createSignedDistanceBetween(ranges.first, ranges.second);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  SegmentedPlanesTerrainModel::SegmentedPlanesTerrainModel(
    PlanarTerrain&& planarTerrain, std::string referenceFrameName): 
    TerrainModel(std::move(referenceFrameName)),
    planarTerrain_(std::move(planarTerrain)),
    signedDistanceField_(nullptr),
    elevationData_(&planarTerrain_.gridMap.get(elevationLayerName))
  {
    planarTerrain_.gridMap.setFrameId(referenceFrameName_);
    const auto ranges = getSignedDistanceRange(planarTerrain_.gridMap, elevationLayerName);
    createSignedDistanceBetween(ranges.first, ranges.second);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  SegmentedPlanesTerrainModel* SegmentedPlanesTerrainModel::clone() const
  {
    return new SegmentedPlanesTerrainModel(*this);
  }
  
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  TerrainPlane SegmentedPlanesTerrainModel::getLocalTerrainAtPositionInWorldAlongGravity(
    const vector3_t& positionInWorld, 
    std::function<scalar_t(const vector3_t&)> penaltyFunction) const
  {
    if(isOutsideGridMap(positionInWorld) || planarTerrain_.planarRegions.empty())
    {
      return getFlatGroundPlane(positionInWorld);
    }

    const auto projection = getBestPlanarRegionAtPositionInWorld(positionInWorld,
      planarTerrain_.planarRegions, std::move(penaltyFunction));

    if(projection.regionPtr == nullptr)
    {
      throw std::runtime_error("[SegmentedPlanesTerrainModel]: No region found");
    }
      return TerrainPlane{projection.positionInWorld, 
        projection.regionPtr->transformPlaneToWorld.linear().transpose()
    };
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ConvexTerrain SegmentedPlanesTerrainModel::getConvexTerrainAtPositionInWorld(
    const vector3_t& positionInWorld, 
    std::function<scalar_t(const vector3_t&)> penaltyFunction) const
  {
    if(isOutsideGridMap(positionInWorld) || planarTerrain_.planarRegions.empty())
    {
      // Flat ground has no boundary, so foothold is not constrained
      return ConvexTerrain(getFlatGroundPlane(positionInWorld), {});
    }

    const auto projection = getBestPlanarRegionAtPositionInWorld(positionInWorld,
      planarTerrain_.planarRegions, std::move(penaltyFunction));

    if (projection.regionPtr == nullptr)
    {
      std::stringstream stringStream;
      stringStream << positionInWorld.transpose();

      std::string message = "[SegmentedPlanesTerrainModel] No region found at: " + stringStream.str();
      throw std::runtime_error(message.c_str());
    }

    // Convert boundary and projection to terrain frame
    const int numberOfVertices = 16; // Multiple of 4 is nice for symmetry.
    const double growthFactor = 1.05;
    const auto convexRegion = growConvexPolygonInsideShape(
      projection.regionPtr->boundaryWithInset.boundary, 
      projection.positionInTerrainFrame, numberOfVertices, growthFactor);

    TerrainPlane convexPlane(projection.positionInWorld,
      projection.regionPtr->transformPlaneToWorld.linear().transpose());
    // Origin is at the projection

    std::vector<vector2_t> convexBoundries;
    convexBoundries.reserve(convexRegion.size());
    for(const auto& point : convexRegion)
    {
      convexBoundries.emplace_back(point.x() - projection.positionInTerrainFrame.x(),
        point.y() - projection.positionInTerrainFrame.y());
      // Shift points to new origin
    }

    // Region growing returns all points at the projection if it could not grow (zero area),
    // such boundary has no edges, so foothold is left unconstrained
    scalar_t doubleArea = 0.0;
    for(size_t i = 0; i < convexBoundries.size(); ++i)
    {
      const auto& currentPoint = convexBoundries[i];
      const auto& nextPoint = convexBoundries[(i + 1) % convexBoundries.size()];
      doubleArea += currentPoint.x() * nextPoint.y() - nextPoint.x() * currentPoint.y();
    }
    const scalar_t minimalArea = 1e-9; // [m^2]
    if(!(std::abs(0.5 * doubleArea) > minimalArea))
    {
      convexBoundries.clear();
    }

    // Return convex region with origin at the projection
    return ConvexTerrain(std::move(convexPlane), std::move(convexBoundries));
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  vector3_t SegmentedPlanesTerrainModel::getHighestObstacleAlongLine(
    const vector3_t& position1InWorld,
    const vector3_t& position2InWorld) const
  {
    // Outside of the map ground is flat, so only part of the line inside of the map is checked
    vector2_t clippedStart, clippedEnd;
    if(clipLineToGridMap(planarTerrain_.gridMap, position1InWorld.head<2>(),
      position2InWorld.head<2>(), clippedStart, clippedEnd))
    {
      const auto result = grid_map::lookup::maxValueBetweenLocations(clippedStart, clippedEnd,
        planarTerrain_.gridMap, *elevationData_);

      if (result.isValid)
      {
        return {result.position.x(), result.position.y(), result.value};
      }
    }

    // return highest query point if the map didn't work.
    if (position1InWorld.z() > position2InWorld.z())
    {
        return position1InWorld;
    }
    else
    {
        return position2InWorld;
    }
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  std::vector<vector2_t> SegmentedPlanesTerrainModel::getHeightProfileAlongLine(
    const vector3_t& position1InWorld,
    const vector3_t& position2InWorld) const
  {
    const vector2_t diff2d = position2InWorld.head<2>() - position1InWorld.head<2>();
    const scalar_t diffSquareNorm = diff2d.squaredNorm();
    const auto resolution = planarTerrain_.gridMap.getResolution();
    if (diffSquareNorm > (resolution * resolution))
    {
      // norm(p2-p1)_XY > resolution
      // Outside of the map ground is flat, so only part of the line inside of the map is sampled
      vector2_t clippedStart, clippedEnd;
      if(!clipLineToGridMap(planarTerrain_.gridMap, position1InWorld.head<2>(),
        position2InWorld.head<2>(), clippedStart, clippedEnd))
      {
        return {};
      }

      const auto pointsOnLine = grid_map::lookup::valuesBetweenLocations(clippedStart, clippedEnd,
        planarTerrain_.gridMap, *elevationData_);
      std::vector<vector2_t> heightProfile;
      heightProfile.reserve(pointsOnLine.size());
      for (const auto& point : pointsOnLine)
      {
          const vector2_t pointDiff = point.head<2>() - position1InWorld.head<2>();
          const scalar_t lineProgress = pointDiff.dot(diff2d) / diffSquareNorm;
          if (lineProgress >= 0.0 && lineProgress <= 1.0)
          {
            heightProfile.push_back({lineProgress, point.z()});
          }
      }
      return heightProfile;
    }
    else
    {
      grid_map::Index index;
      if(!planarTerrain_.gridMap.getIndex({position1InWorld.x(), position1InWorld.y()}, index))
      {
        // Outside of the map ground is flat, so there are no obstacles
        return {};
      }
      scalar_t heightData = (*elevationData_)(index(0), index(1));
      return {{0.0, heightData}, {1.0, heightData}};
    }
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  const SignedDistanceField* SegmentedPlanesTerrainModel::getSignedDistanceField() const
  {
    return signedDistanceField_.get();
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  vector3_t SegmentedPlanesTerrainModel::getSmoothedPositon(
    const vector2_t& positionXYInWorld) const
  {
    return vector3_t(positionXYInWorld.x(), positionXYInWorld.y(),
      getGroundHeightAtClosestCell(planarTerrain_.gridMap, positionXYInWorld));
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  const grid_map::GridMap& SegmentedPlanesTerrainModel::getGridMapTerrain() const
  {
    return planarTerrain_.gridMap;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void SegmentedPlanesTerrainModel::createSignedDistanceBetween(
    const Eigen::Vector3d& minCoordinates, const Eigen::Vector3d& maxCoordinates)
  {
    // Compute coordinates of submap
    const auto minXY = grid_map::lookup::projectToMapWithMargin(planarTerrain_.gridMap,
      grid_map::Position(minCoordinates.x(), minCoordinates.y()));
    const auto maxXY = grid_map::lookup::projectToMapWithMargin(planarTerrain_.gridMap,
      grid_map::Position(maxCoordinates.x(), maxCoordinates.y()));

    const auto centerXY = 0.5 * (minXY + maxXY);
    const auto lengths = maxXY - minXY;
    bool success = true;
    grid_map::GridMap subMap = planarTerrain_.gridMap.getSubmap(centerXY, lengths, success);
    if (success)
    {
      signedDistanceField_ = std::make_unique<SegmentedPlanesSignedDistanceField>(subMap, 
        elevationLayerName, minCoordinates.z(), maxCoordinates.z());
    }
    else
    {
      throw std::runtime_error("[SegmentedPlanesTerrainModel]: Failed to get subMap");
    }
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  SegmentedPlanesTerrainModel::SegmentedPlanesTerrainModel(
    const SegmentedPlanesTerrainModel& rhs):
    TerrainModel(rhs),
    planarTerrain_(rhs.planarTerrain_),
    signedDistanceField_(new SegmentedPlanesSignedDistanceField(*rhs.signedDistanceField_)),
    elevationData_(&planarTerrain_.gridMap.get(elevationLayerName)) {}
  
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  std::pair<Eigen::Vector3d, Eigen::Vector3d> SegmentedPlanesTerrainModel::getSignedDistanceRange(
    const grid_map::GridMap& gridMap, const std::string& elevationLayer)
  {
    // Read min-max from elevation map
    const float heightMargin = 0.5;
    const auto& elevationData = gridMap.get(elevationLayer);
    const float minValue = elevationData.minCoeffOfFinites() - heightMargin;
    const float maxValue = elevationData.maxCoeffOfFinites() + heightMargin;
    auto minXY = grid_map::lookup::projectToMapWithMargin(
        gridMap, grid_map::Position(std::numeric_limits<double>::lowest(),
                                    std::numeric_limits<double>::lowest()));
    auto maxXY = grid_map::lookup::projectToMapWithMargin(
        gridMap, grid_map::Position(std::numeric_limits<double>::max(),
                                      std::numeric_limits<double>::max()));

    const vector3_t minCoordinates{minXY.x(), minXY.y(), minValue};
    const vector3_t maxCoordinates{maxXY.x(), maxXY.y(), maxValue};

    return {minCoordinates, maxCoordinates};
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  TerrainPlane SegmentedPlanesTerrainModel::getFlatGroundPlane(const vector3_t& positionInWorld) const
  {
    // Same ground height as used for base reference
    return TerrainPlane(getSmoothedPositon(positionInWorld.head<2>()), matrix3_t::Identity());
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  bool SegmentedPlanesTerrainModel::isOutsideGridMap(const vector3_t& positionInWorld) const
  {
    return !planarTerrain_.gridMap.isInside(positionInWorld.head<2>());
  }
} // namespace terrain_model
