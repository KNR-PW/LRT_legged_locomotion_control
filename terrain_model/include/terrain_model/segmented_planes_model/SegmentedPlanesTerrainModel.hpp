//
// Created by rgrandia on 23.06.20.
// Modified by Bartłomiej Krajewski (https://github.com/BartlomiejK2) on 30.04.2026 
//

#ifndef __SEGMENTED_PLANES_TERRAIN_MODEL_TERRAIN_MODEL__
#define __SEGMENTED_PLANES_TERRAIN_MODEL_TERRAIN_MODEL__

#include <memory>

#include <terrain_model/core/Types.hpp>
#include <terrain_model/core/TerrainModel.hpp>
#include <terrain_model/segmented_planes_model/SegmentedPlanesSignedDistanceField.hpp>

#include <convex_plane_decomposition/PlanarRegion.h>

namespace terrain_model
{
  /**
   * Terrain model made of segmented planar regions. Outside of the grid map (and when
   * there are no planar regions) terrain is flat ground at the height of the nearest map
   * edge, taken from the ground layer (see SegmentedPlanesGridMapUtils.hpp).
   * Only elevation and ground layers of the grid map are kept.
   */
  class SegmentedPlanesTerrainModel : public TerrainModel
  {
    public:

      SegmentedPlanesTerrainModel(
        const convex_plane_decomposition::PlanarTerrain& planarTerrain, 
        std::string referenceFrameName = "world");

      SegmentedPlanesTerrainModel(
        convex_plane_decomposition::PlanarTerrain&& planarTerrain, 
        std::string referenceFrameName = "world");

      ~SegmentedPlanesTerrainModel() override = default;

      SegmentedPlanesTerrainModel* clone() const override;

      // Keep overloads without penalty function from the interface visible
      using TerrainModel::getLocalTerrainAtPositionInWorldAlongGravity;
      using TerrainModel::getConvexTerrainAtPositionInWorld;

      TerrainPlane getLocalTerrainAtPositionInWorldAlongGravity(
        const vector3_t& positionInWorld,
        std::function<ocs2::scalar_t(const vector3_t&)> penaltyFunction) const override;

      vector3_t getHighestObstacleAlongLine(
        const vector3_t& position1InWorld,
        const vector3_t& position2InWorld) const override;

      std::vector<vector2_t> getHeightProfileAlongLine(
        const vector3_t& position1InWorld,
        const vector3_t& position2InWorld) const override;

      ConvexTerrain getConvexTerrainAtPositionInWorld(
        const vector3_t& positionInWorld,
        std::function<ocs2::scalar_t(const vector3_t&)> penaltyFunction) const override;

      const SignedDistanceField* getSignedDistanceField() const override;

      vector3_t getSmoothedPosition(const vector2_t& positionXYInWorld) const override;

      const grid_map::GridMap& getGridMapTerrain() const override;

    private:

      /** Copy shares the terrain data, it is never modified after construction */
      SegmentedPlanesTerrainModel(const SegmentedPlanesTerrainModel& rhs) = default;

      /** Horizontal plane at the height of the closest grid map cell */
      TerrainPlane getFlatGroundPlane(const vector3_t& positionInWorld) const;

      bool isOutsideGridMap(const vector3_t& positionInWorld) const;

      // Shared between clones, so cloning does not copy the grid map and distance field
      std::shared_ptr<const convex_plane_decomposition::PlanarTerrain> planarTerrain_;
      std::shared_ptr<const SegmentedPlanesSignedDistanceField> signedDistanceField_;

      // Layer data inside of planarTerrain_
      const grid_map::Matrix* const elevationData_;
      const grid_map::Matrix* const groundData_;
  };
} // namespace terrain_model

#endif