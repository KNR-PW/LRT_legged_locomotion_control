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


#ifndef __GAIT_DYNAMIC_PHASE_LEGGED_LOCOMOTION_MPC__
#define __GAIT_DYNAMIC_PHASE_LEGGED_LOCOMOTION_MPC__

#include <legged_locomotion_mpc/common/Types.hpp>
#include <legged_locomotion_mpc/locomotion/GaitParameters.hpp>

namespace legged_locomotion_mpc
{
  namespace locomotion
  {
    class GaitDynamicPhaseController
    {
      public:
        GaitDynamicPhaseController(ocs2::scalar_t initTime,
          const GaitStaticParameters& staticParams,
          const GaitDynamicParameters& initDynamicParams);

        /**
         * Get phase values at given query time
         */
        std::vector<ocs2::scalar_t> getPhasesAtTime(ocs2::scalar_t time) const;

        /**
         * Get contact flags at given query time
         */
        contact_flags_t getContactFlagsAtTime(ocs2::scalar_t time) const;

        const GaitDynamicParameters& getDynamicParametersAtTime(ocs2::scalar_t time) const;

        /**
         * Get base phase (phase of the first end effector) just after query time. It differs from
         * getPhasesAtTime() only at the start of segment with new phase (see update() with phase)
         */
        ocs2::scalar_t getBasePhaseAfterTime(ocs2::scalar_t time) const;

        /**
         * Remove event times, frequencies and offsets before query time
         */
        void remove(ocs2::scalar_t time);

        /**
         * Add new frequency and offsets, starting from query time
         */
        void update(ocs2::scalar_t newTime, const GaitDynamicParameters& newDynamicParams);
        void update(ocs2::scalar_t newTime, GaitDynamicParameters&& newDynamicParams);

        /**
         * Add new frequency and offsets, starting from query time with given base phase (phase jump).
         * At query time phases are still from the previous segment (same as mode at event time
         * in ModeSchedule), new phase is used after it.
         */
        void update(ocs2::scalar_t newTime, const GaitDynamicParameters& newDynamicParams,
          ocs2::scalar_t newBasePhase);
        
      private:

        GaitStaticParameters staticParams_;

        /**
         * 
         * Data is counted as follows:
         * 
         *          ------ | ------ | ------ | ...  ------ | ------ | ------
         *  time:         t[0]     t[1]     t[2]       t[n-1]     t[n]
         *  frequency:        f[0]     f[1]     f[2] ...    f[n-1]     f[n]
         *  offsets:        o[0]][:]  o[1]][:] o[2]][:]   o[n-1]][:]  o[n]][:]
         *  swing ratio:      s[0]     s[1]     s[2]        s[n-1]     s[n]
         *  cached phase: p[0]     p[1]     p[2]       p[n-1]     p[n]
         *
         * Where t[0] should have same value as ModeSchedule.eventTimes[0]!
         */
        std::vector<ocs2::scalar_t> eventTimes_;
        std::vector<GaitDynamicParameters> dynamicParamsVec_;
        
        /**
         * Cached phase values, starting from value at time t[0]
         */
        std::vector<ocs2::scalar_t> cachedPhase_;
    };
  } // namespace locomotion
} // namespace legged_locomotion_mpc

#endif