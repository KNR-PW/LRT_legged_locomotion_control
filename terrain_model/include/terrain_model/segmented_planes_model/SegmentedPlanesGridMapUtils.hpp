// Copyright (c) 2025, Koło Naukowe Robotyków
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.


#ifndef __SEGMENTED_PLANES_GRID_MAP_UTILS_TERRAIN_MODEL__
#define __SEGMENTED_PLANES_GRID_MAP_UTILS_TERRAIN_MODEL__

#include <string>

#include <grid_map_core/GridMap.hpp>

#include <terrain_model/core/Types.hpp>

namespace terrain_model
{
  /**
   * Layer with smoothed height of planar regions, made by plane decomposition pipeline.
   * Outside of the grid map terrain is flat ground at the height of this layer
   * at the nearest map edge.
   */
  const std::string groundLayerName = "smooth_planar";

  /**
   * Get ground height at the closest grid map cell. For position outside of the map
   * it is the height of the nearest edge cell, so ground continues flat outside of the map.
   * @param [in] gridMap: grid map with ground layer
   * @param [in] positionXYInWorld: 2D query position in world frame
   * @return ground height in world frame, 0.0 if ground layer value is not finite
   */
  ocs2::scalar_t getGroundHeightAtClosestCell(const grid_map::GridMap& gridMap,
    const vector2_t& positionXYInWorld);

  /**
   * Clip line between two points to the part that is inside of the grid map
   * @param [in] gridMap: grid map
   * @param [in] start: 2D start of the line in world frame
   * @param [in] end: 2D end of the line in world frame
   * @param [out] clippedStart: 2D start of the clipped line in world frame
   * @param [out] clippedEnd: 2D end of the clipped line in world frame
   * @return true if part of the line is inside of the map
   */
  bool clipLineToGridMap(const grid_map::GridMap& gridMap, const vector2_t& start,
    const vector2_t& end, vector2_t& clippedStart, vector2_t& clippedEnd);
}; // namespace terrain_model

#endif
