//
// Created by rgrandia on 29.04.20.
// Modified by Bartłomiej Krajewski (https://github.com/BartlomiejK2) on 29.08.2025 
//

#ifndef __PLANAR_TERRAIN_MODEL_TERRAIN_MODEL__
#define __PLANAR_TERRAIN_MODEL_TERRAIN_MODEL__

#include <terrain_model/core/Types.hpp>
#include <terrain_model/core/TerrainModel.hpp>
#include <terrain_model/planar_model/PlanarSignedDistanceField.hpp>

namespace terrain_model
{
  /**
   * This class models the terrain as a single infinite plane.
   */
  class PlanarTerrainModel : public TerrainModel
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      // Static const parameters for GridMap representation of plane terrain
      static constexpr ocs2::scalar_t GRIDMAP_LENGTH = 3.0;     // [m]
      static constexpr ocs2::scalar_t GRIDMAP_RESOLUTION = 0.5; // [m/cell]

      PlanarTerrainModel(TerrainPlane terrainPlane, 
        std::string referenceFrameName = "world");

      ~PlanarTerrainModel() override = default;

      PlanarTerrainModel* clone() const override;

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

      vector3_t getSmoothedPositon(const vector2_t& positionXYInWorld) const override;

      const grid_map::GridMap& getGridMapTerrain() const override;

    private:

      TerrainPlane terrainPlane_;
      PlanarSignedDistanceField sdf_;
      grid_map::GridMap gridMap_;
  };
}; // namespace terrain_model

#endif