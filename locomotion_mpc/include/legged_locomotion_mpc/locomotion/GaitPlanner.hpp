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

#include <optional>
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

        /**
         * Sets target dynamic parameters, the first step towards them (see stepDynamicParameters())
         * is done at time.
         * @param [in] time: time of the change
         * @param [in] dynamicParams: target dynamic parameters
         */
        void updateDynamicParameters(ocs2::scalar_t time,
          const GaitDynamicParameters& dynamicParams);

        /**
         * Moves applied dynamic parameters towards the target, has to be called with increasing time
         * (e.g. in every MPC iteration).
         *
         * Start (stepping frequency from minimum), stop (to minimum) and gait type change (phase offsets)
         * are applied at once with new base phase (re-phasing), chosen so that contact states of
         * end effectors at time are kept (if possible, otherwise as few of them as possible are changed).
         * E.g. walking from standing starts with end effectors in contact. Stop keeps as many
         * end effectors in contact as possible. Standing robot does not lift end effectors, change that
         * would lift them waits for start.
         *
         * Otherwise stepping frequency and swing ratio are moved towards the target with maximum rates
         * of static parameters. Swing ratio is changed only so far, that no end effector changes its
         * contact state at time (end effector that touched down would lift off again, swinging end
         * effector would touch down in the air). The rest of the change is done in next steps.
         * @param [in] time: time of the change
         */
        void stepDynamicParameters(ocs2::scalar_t time);
        
        GaitFlags updateCurrentContacts(ocs2::scalar_t time, 
          const contact_flags_t& currentContacts);

        std::vector<ocs2::scalar_t> getPhasesAtTime(ocs2::scalar_t time) const;

        std::vector<std::vector<ocs2::scalar_t>> getPhasesAtTimes(
          const std::vector<ocs2::scalar_t>& times) const;

        contact_flags_t getContactFlagsAtTime(ocs2::scalar_t time) const;

        std::vector<contact_flags_t> getContactFlagsAtTimes(
          const std::vector<ocs2::scalar_t>& times) const;

        const GaitStaticParameters& getStaticParameters();

        /** Applied dynamic parameters (latest ones used in the mode schedule) */
        const GaitDynamicParameters& getDynamicParameters();

        /** Target dynamic parameters (latest from updateDynamicParameters()) */
        const GaitDynamicParameters& getTargetDynamicParameters() const;

      private:

        GaitDynamicParameters clampGaitDynamicParams(
          const GaitDynamicParameters& dynamicParams);

        /**
         * Applies dynamic parameters at time, there is no rate limit
         * @param [in] time: time of the change
         * @param [in] clampedParams: new dynamic parameters
         * @param [in] newBasePhase: base phase after time (re-phasing), phase is continuous when not set
         */
        void applyDynamicParameters(ocs2::scalar_t time,
          const GaitDynamicParameters& clampedParams,
          std::optional<ocs2::scalar_t> newBasePhase = std::nullopt);

        /** Criterion of new base phase for start, stop and gait type change */
        enum class RephasingMode
        {
          WALKING,  // Start or gait type change: as few contact changes as possible, then most contacts
          STOP,     // Most end effectors in contact, then as few contact changes as possible
          STANDING  // As STOP, but no end effector is lifted
        };

        /**
         * New base phase at time for new dynamic parameters. Phase is not changed, when contact states at
         * time are already the best ones. Otherwise the earliest phase of the best contact states
         * (end effectors that changed contact state at once are at the start of the new state),
         * with margin, so that the next contact change is not too close.
         * @return new base phase, nullopt when there is no valid one (STANDING)
         */
        std::optional<ocs2::scalar_t> getRephasedBasePhase(ocs2::scalar_t time,
          const GaitDynamicParameters& newParams, RephasingMode mode) const;

        /** Stepping frequency is at or below minimum, robot is standing in place */
        bool isStanding(ocs2::scalar_t steppingFrequency) const;

        /**
         * Swing ratio between current and new one, the closest to the new one for which no end
         * effector changes its contact state at time (contact: phase >= swing ratio).
         */
        ocs2::scalar_t getContactPreservingSwingRatio(ocs2::scalar_t time,
          ocs2::scalar_t currentSwingRatio, ocs2::scalar_t newSwingRatio,
          const std::vector<ocs2::scalar_t>& phaseOffsets) const;

        const GaitStaticParameters staticParams_;

        ocs2::scalar_t currentDynamicParamsTime_;
        GaitDynamicParameters dynamicParams_;

        // Target of rate limited changes and time of their last step
        GaitDynamicParameters targetDynamicParams_;
        ocs2::scalar_t lastStepTime_;

        ocs2::ModeSchedule modeSchedule_;

        GaitDynamicPhaseController phaseController_;

        ocs2::scalar_t currentStartTime_;

    };
  } // namespace locomotion
} // namespace legged_locomotion_mpc

#endif