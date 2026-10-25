#include <terrain_model/segmented_planes_model/SegmentedPlanesSignedDistanceField.hpp>
#include <terrain_model/segmented_planes_model/SegmentedPlanesGridMapUtils.hpp>

#include <algorithm>
#include <cmath>

namespace terrain_model
{
  using namespace ocs2;
  using namespace grid_map;
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  SegmentedPlanesSignedDistanceField::SegmentedPlanesSignedDistanceField(
    const grid_map::GridMap& gridMap, const std::string& elevationLayer,
    scalar_t /* minHeight */, scalar_t maxHeight):
      groundMap_(std::vector<std::string>{groundLayerName}),
      hasElevationData_(false)
  {
    // Copy only ground layer, it is used as flat ground outside of the map
    groundMap_.setGeometry(gridMap.getLength(), gridMap.getResolution(), gridMap.getPosition());
    groundMap_.setStartIndex(gridMap.getStartIndex());
    groundMap_.get(groundLayerName) = gridMap.get(groundLayerName);

    // grid_map_sdf ends height layers at heightClearance above the highest cell
    const scalar_t maxElevation = gridMap.get(elevationLayer).maxCoeffOfFinites();
    hasElevationData_ = std::isfinite(maxElevation);
    if(hasElevationData_)
    {
      // At least 3 height layers are needed for the gradient (also if maxHeight is not finite)
      const scalar_t heightClearance = std::max(3.0 * gridMap.getResolution(),
        maxHeight - maxElevation);
      sdf_.calculateSignedDistanceField(gridMap, elevationLayer, heightClearance);
    }
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  SegmentedPlanesSignedDistanceField* SegmentedPlanesSignedDistanceField::clone() const 
  {
    return new SegmentedPlanesSignedDistanceField(*this);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  scalar_t SegmentedPlanesSignedDistanceField::value(const vector3_t& position) const
  {
    if(!hasElevationData_ || !groundMap_.isInside(position.head<2>()))
    {
      // Outside of the map terrain is flat ground at the height of the nearest map edge
      return position.z() - getGroundHeightAtClosestCell(groundMap_, position.head<2>());
    }
    return sdf_.getDistanceAt(position);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  vector3_t SegmentedPlanesSignedDistanceField::derivative(const vector3_t& position) const
  {
    if(!hasElevationData_ || !groundMap_.isInside(position.head<2>()))
    {
      // Flat ground outside of the map
      return vector3_t::UnitZ();
    }
    return sdf_.getDistanceGradientAt(position);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  std::pair<scalar_t, vector3_t> SegmentedPlanesSignedDistanceField::valueAndDerivative(
    const vector3_t& position) const
  {
    return {value(position), derivative(position)};
  }
} // namespace terrain_model