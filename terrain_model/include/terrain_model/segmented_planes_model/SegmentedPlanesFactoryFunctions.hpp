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

/*
 * Authors: Bartłomiej Krajewski (https://github.com/BartlomiejK2)
 */


#ifndef __SEGMENTED_PLANES_FACTORY_FUNCTIONS_TERRAIN_MODEL__
#define __SEGMENTED_PLANES_FACTORY_FUNCTIONS_TERRAIN_MODEL__

#include <terrain_model/core/Types.hpp>
#include <terrain_model/core/SignedDistanceField.hpp>

#include <convex_plane_decomposition/PlanarRegion.h>
#include <convex_plane_decomposition/PlaneDecompositionPipeline.h>

namespace terrain_model
{
  /**
   * Runs plane decomposition pipeline on elevation map
   * @param [in] gridMap: elevation map, it is moved into the result
   * @param [in] pipelineConfig: configuration of the plane decomposition pipeline
   * @param [in] elevationLayerName: name of the elevation layer
   * @return planar terrain (planar regions and processed grid map)
   */
  convex_plane_decomposition::PlanarTerrain computePlanarTerrain(
    grid_map::GridMap&& gridMap,
    const convex_plane_decomposition::PlaneDecompositionPipeline::Config& pipelineConfig,
    const std::string& elevationLayerName = terrain_model::elevationLayerName);

}; // namespace terrain_model

#endif