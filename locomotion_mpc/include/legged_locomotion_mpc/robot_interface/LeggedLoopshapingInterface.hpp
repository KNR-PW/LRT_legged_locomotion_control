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

#ifndef __LEGGED_LOOPSHAPING_INTERFACE_LEGGED_LOCOMOTION_MPC__
#define __LEGGED_LOOPSHAPING_INTERFACE_LEGGED_LOCOMOTION_MPC__

#include <ocs2_robotic_tools/common/LoopshapingRobotInterface.h>
#include <ocs2_oc/synchronized_module/LoopshapingSynchronizedModule.h>

#include <legged_locomotion_mpc/robot_interface/LeggedInterface.hpp>

namespace legged_locomotion_mpc
{
  class LeggedLoopshapingInterface: public ocs2::LoopshapingRobotInterface
  {
    public:

      LeggedLoopshapingInterface(std::unique_ptr<LeggedInterface> leggedInterfacePtr,
        std::shared_ptr<ocs2::LoopshapingDefinition> loopshapingDefinitionPtr);
      
      /**
       * Gets normal legged interface
       * @return reference to the LeggedInterface
       * @warning While using loopshaping interface, do not use normal interface for
       * getting optimal problem, reference manager, rollout and initial state for
       * MPC, if your using it to change some parameters or references it is fine
       */
      LeggedInterface& getLeggedInterface();
      
      /**
       * Gets the loopshaping initial state (system + filter)
       * @return const reference to the loopshaping initial state (system + filter)
       */
      const ocs2::vector_t& getInitialState() const;

      /**
       * Gets the loopshaping Rollout.
       * @return reference to the loopshaping RolloutBase.
       */
      ocs2::RolloutBase& getRollout();

      /**
       * Gets the loopshaping disturbance synchronized module
       * @return shared pointer to the loopshaping disturbance synchronized module
       */
      std::shared_ptr<ocs2::LoopshapingSynchronizedModule>  getSynchronizedModules();

    private:
      
      ocs2::vector_t initialState_;

      std::unique_ptr<ocs2::RolloutBase> loopshapingRolloutPtr_;

      std::shared_ptr<ocs2::LoopshapingSynchronizedModule> loopshapingDisturbanceModule_;
  };

  LeggedLoopshapingInterface makeLeggedLoopshapingInterface(
    ocs2::scalar_t initTime, const ocs2::vector_t& currentSystemState, 
    std::unique_ptr<terrain_model::TerrainModel> currentTerrainModel, 
    const std::string& taskFile, const std::string& modelFile, const std::string& urdfFile, 
    const std::string& loopshapingDefinitionFile);

  std::unique_ptr<LeggedLoopshapingInterface> makeLeggedLoopshapingInterfacePointer(
    ocs2::scalar_t initTime, const ocs2::vector_t& currentSystemState, 
    std::unique_ptr<terrain_model::TerrainModel> currentTerrainModel, 
    const std::string& taskFile, const std::string& modelFile, const std::string& urdfFile, 
    const std::string& loopshapingDefinitionFile);
} // namespace legged_locomotion_mpc

#endif