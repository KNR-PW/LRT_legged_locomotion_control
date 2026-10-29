#include <terrain_model/segmented_planes_model/SegmentedPlanesGridMapUtils.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

#include <grid_map_filters_rsl/lookup.hpp>

namespace terrain_model
{
  using namespace ocs2;
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  scalar_t getGroundHeightAtClosestCell(const grid_map::GridMap& gridMap,
    const vector2_t& positionXYInWorld)
  {
    return getGroundHeightAtClosestCell(gridMap, gridMap.get(groundLayerName), positionXYInWorld);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  scalar_t getGroundHeightAtClosestCell(const grid_map::GridMap& gridMap,
    const grid_map::Matrix& groundData, const vector2_t& positionXYInWorld)
  {
    // Half of the cell margin keeps clamped position inside of the edge cell
    const grid_map::Position clampedPosition = grid_map::lookup::projectToMapWithMargin(
      gridMap, positionXYInWorld, 0.5 * gridMap.getResolution());

    grid_map::Index index;
    if(!gridMap.getIndex(clampedPosition, index))
    {
      return 0.0;
    }

    const scalar_t height = groundData(index(0), index(1));
    return std::isfinite(height) ? height : 0.0;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  bool clipLineToGridMap(const grid_map::GridMap& gridMap, const vector2_t& start,
    const vector2_t& end, vector2_t& clippedStart, vector2_t& clippedEnd)
  {
    if(!start.allFinite() || !end.allFinite())
    {
      return false;
    }

    // Half of the cell margin keeps clipped points inside of the edge cells
    const vector2_t halfLength = 0.5 * gridMap.getLength().matrix()
      - vector2_t::Constant(0.5 * gridMap.getResolution());
    const vector2_t lowerBound = gridMap.getPosition() - halfLength;
    const vector2_t upperBound = gridMap.getPosition() + halfLength;

    // Liang-Barsky clipping of line: start + t * (end - start), t in [0, 1]
    const vector2_t direction = end - start;
    scalar_t tMin = 0.0;
    scalar_t tMax = 1.0;
    for(size_t i = 0; i < 2; ++i)
    {
      if(std::abs(direction(i)) < std::numeric_limits<scalar_t>::epsilon())
      {
        // Line is parallel to this axis, so it has to be between bounds
        if(start(i) < lowerBound(i) || start(i) > upperBound(i))
        {
          return false;
        }
      }
      else
      {
        scalar_t tLower = (lowerBound(i) - start(i)) / direction(i);
        scalar_t tUpper = (upperBound(i) - start(i)) / direction(i);
        if(tLower > tUpper)
        {
          std::swap(tLower, tUpper);
        }
        tMin = std::max(tMin, tLower);
        tMax = std::min(tMax, tUpper);
        if(tMin > tMax)
        {
          return false;
        }
      }
    }

    clippedStart = start + tMin * direction;
    clippedEnd = start + tMax * direction;
    return true;
  }
} // namespace terrain_model
