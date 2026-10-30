// Copyright (c) 2025, Bartłomiej Krajewski
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

#ifndef __COLLISION_SETTINGS_LEGGED_LOCOMOTION_MPC__
#define __COLLISION_SETTINGS_LEGGED_LOCOMOTION_MPC__

#include <ocs2_core/Types.h>

#include <legged_locomotion_mpc/common/Types.hpp>
#include <legged_locomotion_mpc/common/ModelSettings.hpp>

namespace legged_locomotion_mpc
{
  namespace collision
  {
    struct CollisionSettings
    {
      /**
       *  Other collision links for terrain or self collision (without end effectors!)
       */ 
      std::vector<std::string> collisionLinkNames;

      /**
       * Terrain collision links (without end effectors!)
       */
      std::vector<std::string> terrainCollisionLinkNames;

      /**
       * Self collision link pairs (including end efffectors!)
       */
      std::vector<std::pair<std::string, std::string>> selfCollisionPairNames;

      /**
       * vector of maximum allowed distances between the surfaces 
       * of the collision primitives and collision spheres
       */
      std::vector<ocs2::scalar_t> maxExcesses;

      /**
       * Relaxation values for terrain avoidance 
       * Size: Number of end effectors + all other terrain collision names
       */
      std::vector<ocs2::scalar_t> terrainRelaxations;

      /**
       * Relaxation for self collision avoidance
       */
      std::vector<ocs2::scalar_t> selfCollisionRelaxations;

      /**
       * shrinking ratio for maxExcess to recursively approximate the circular base 
       * of the cylinder when more than one collision 
       * sphere is required along the radial direction
       */
      ocs2::scalar_t shrinkRatio = 0.75;

      /**
       * Maximum number of sphere neighbours for single sphere (including itself)
       */
      size_t maxSphereNeighbours = 5;
    };

  /**
   * Creates MPC CollisionSettings 
   * @param [in] filename: file path with collision settings.
   * @param [in] fieldName: field where settings are defined
   * @param [in] verbose: verbose flag
   * @return CollisionSettings struct
   */
  CollisionSettings loadCollisionSettings(const std::string& filename,
    const ModelSettings& modelSettings,
    const std::string& fieldName = "legged_collision_settings",
    bool verbose = true);
  } // namespace collision
} // namespace legged_locomotion_mpc

#endif