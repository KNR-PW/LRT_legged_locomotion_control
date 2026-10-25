//
// Created by rgrandia on 17.03.22.
// Modified by Bartłomiej Krajewski (https://github.com/BartlomiejK2) on 30.04.2026 
//

#ifndef __SEGMENTED_PLANES_SIGNED_DISTANCE_FIELD_TERRAIN_MODEL__
#define __SEGMENTED_PLANES_SIGNED_DISTANCE_FIELD_TERRAIN_MODEL__

#include <terrain_model/core/Types.hpp>
#include <terrain_model/core/SignedDistanceField.hpp>

#include <grid_map_core/GridMap.hpp>
#include <grid_map_sdf/SignedDistanceField.hpp>

namespace terrain_model
{
  /**
   * Simple wrapper class to implement the switched_model::SignedDistanceField
   * interface. See the forwarded function for documentation.
   * Outside of the grid map terrain is flat ground at the height of the nearest
   * map edge (see SegmentedPlanesGridMapUtils.hpp).
   */
  class SegmentedPlanesSignedDistanceField : public SignedDistanceField 
  {
    public:

      /**
       * @param [in] gridMap: grid map with elevation and ground layer
       * @param [in] elevationLayer: name of the elevation layer
       * @param [in] minHeight: not used, grid_map_sdf always starts 0.2 m below the lowest cell
       * @param [in] maxHeight: absolute height [m] up to which distance field is computed
       */
      SegmentedPlanesSignedDistanceField(const grid_map::GridMap& gridMap,
        const std::string& elevationLayer, ocs2::scalar_t minHeight, ocs2::scalar_t maxHeight);

      ~SegmentedPlanesSignedDistanceField() override = default;
      
      SegmentedPlanesSignedDistanceField* clone() const override;

      ocs2::scalar_t value(const vector3_t& position) const override;

      vector3_t derivative(const vector3_t& position) const override;

      std::pair<ocs2::scalar_t, vector3_t> valueAndDerivative(
        const vector3_t& position) const override;
      
    private:
      grid_map::SignedDistanceField sdf_;

      /** Grid map with only ground layer, used for flat ground outside of the map */
      grid_map::GridMap groundMap_;

      /** False if elevation has no finite values, then whole terrain is flat ground */
      bool hasElevationData_;
  };
} // namespace terrain_model

#endif