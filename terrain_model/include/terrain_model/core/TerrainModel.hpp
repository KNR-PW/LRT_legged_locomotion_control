//
// Created by rgrandia on 21.04.20.
// Modified by Bartłomiej Krajewski (https://github.com/BartlomiejK2) on 29.08.2025 
//

#ifndef __TERRAIN_MODEL_TERRAIN_MODEL__
#define __TERRAIN_MODEL_TERRAIN_MODEL__

#include <functional>
#include <string>
#include <vector>

#include <grid_map_core/GridMap.hpp>

#include <terrain_model/core/Types.hpp>
#include <terrain_model/core/SignedDistanceField.hpp>
#include <terrain_model/core/TerrainPlane.hpp>
#include <terrain_model/core/ConvexTerrain.hpp>

namespace terrain_model
{
  /**
   * This abstract class defines the interface for terrain models.
   */
  class TerrainModel
  {
    public:
    
      TerrainModel(std::string referenceFrameName = "world"): 
        referenceFrameName_(std::move(referenceFrameName)) {}

      virtual ~TerrainModel() = default;

      virtual TerrainModel* clone() const = 0;

      /** Returns a linear approximation of the terrain at the query point projected along gravity onto the terrain  */
      TerrainPlane getLocalTerrainAtPositionInWorldAlongGravity(
        const vector3_t& positionInWorld) const;

      /** Penalty function needs to return values >= 0 */ 
      virtual TerrainPlane getLocalTerrainAtPositionInWorldAlongGravity(
        const vector3_t& positionInWorld,
        std::function<ocs2::scalar_t(const vector3_t&)> penaltyFunction) const = 0;
      
      ConvexTerrain getConvexTerrainAtPositionInWorld(const vector3_t& positionInWorld) const;
      
      /** Penalty function needs to return values >= 0 */
      virtual ConvexTerrain getConvexTerrainAtPositionInWorld(
        const vector3_t& positionInWorld,
        std::function<ocs2::scalar_t(const vector3_t&)> penaltyFunction) const;
      
      /** Returns the signed distance field for this terrain if one is available */
      virtual const SignedDistanceField* getSignedDistanceField() const;

      virtual vector3_t getHighestObstacleAlongLine(
        const vector3_t& position1InWorld,                                        
        const vector3_t& position2InWorld) const = 0;
      /**
       * Returns the height profiles between two points in world frame. Provided as a set of points {alpha, height}, where alpha in [0, 1] is
       * the progress along the line. position1InWorld -> alpha = 0, position2InWorld -> alpha = 1.
       * Height is the absolute height in world frame.
       * Profile can be empty if there is no terrain data along the line.
       */
      virtual std::vector<vector2_t> getHeightProfileAlongLine(
        const vector3_t& position1InWorld,
        const vector3_t& position2InWorld) const = 0;
      
      /**
       * Return vector with the same X and Y values but new "smoothed" height 
       * value in world frame.
       */
      virtual vector3_t getSmoothedPosition(const vector2_t& positionXYInWorld) const = 0;
        
      /**
       * Get const reference of GridMap representation of terrain model.
       * Data is stored in "elevation" layer
       */
      virtual const grid_map::GridMap& getGridMapTerrain() const = 0;

      protected:

        /** 
         * Name of the frame that terrain position is relative to
         */
        std::string referenceFrameName_;
    };
}; // namespace terrain_model

#endif