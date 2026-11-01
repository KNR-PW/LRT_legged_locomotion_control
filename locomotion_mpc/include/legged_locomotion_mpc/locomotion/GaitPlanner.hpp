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


#ifndef __GAIT_PLANNER_LEGGED_LOCOMOTION_MPC__
#define __GAIT_PLANNER_LEGGED_LOCOMOTION_MPC__

#include <vector>
#include <string>

#include <ocs2_core/Types.h>
#include <ocs2_core/reference/ModeSchedule.h>

#include <legged_locomotion_mpc/locomotion/GaitCommon.hpp>
#include <legged_locomotion_mpc/locomotion/GaitParameters.hpp>
#include <legged_locomotion_mpc/locomotion/GaitDynamicPhaseController.hpp>

namespace legged_locomotion_mpc
{
  namespace locomotion
  {
    class GaitPlanner
    {
      public:

        GaitPlanner(const GaitStaticParameters& staticParams,
          const GaitDynamicParameters& initDynamicParams, ocs2::scalar_t initTime);

        ocs2::ModeSchedule getModeSchedule(ocs2::scalar_t startTime, 
          ocs2::scalar_t finalTime);

        void updateDynamicParameters(ocs2::scalar_t time,
          const GaitDynamicParameters& dynamicParams);
        
        GaitFlags updateCurrentContacts(ocs2::scalar_t time, 
          const contact_flags_t& currentContacts);

        std::vector<ocs2::scalar_t> getPhasesAtTime(ocs2::scalar_t time) const;

        std::vector<std::vector<ocs2::scalar_t>> getPhasesAtTimes(
          const std::vector<ocs2::scalar_t>& times) const;

        contact_flags_t getContactFlagsAtTime(ocs2::scalar_t time) const;

        std::vector<contact_flags_t> getContactFlagsAtTimes(
          const std::vector<ocs2::scalar_t>& times) const;

        const GaitStaticParameters& getStaticParameters();

        const GaitDynamicParameters& getDynamicParameters();

      private:

        GaitDynamicParameters clampGaitDynamicParams(
          const GaitDynamicParameters& dynamicParams);
  
        const GaitStaticParameters staticParams_;

        ocs2::scalar_t currentDynamicParamsTime_;
        GaitDynamicParameters dynamicParams_;

        ocs2::ModeSchedule modeSchedule_;

        GaitDynamicPhaseController phaseController_;

        ocs2::scalar_t currentStartTime_;

    };
  } // namespace locomotion
} // namespace legged_locomotion_mpc

#endif