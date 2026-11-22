// Copyright (c) 2026, Bartłomiej Krajewski
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

#ifndef __CONTACT_FLAGS_FUSION_LEGGED_WHOLE_BODY_CONTROL_ROS2__
#define __CONTACT_FLAGS_FUSION_LEGGED_WHOLE_BODY_CONTROL_ROS2__

#include <algorithm>
#include <cassert>
#include <limits>
#include <vector>

#include <legged_whole_body_control/Types.hpp>

namespace legged_whole_body_control_ros2
{
  /**
   * Time from time to the next planned touchdown of every end effector.
   * Mode modeSequence[k] is active between eventTimes[k - 1] and eventTimes[k], mode at
   * event time is the previous one (as in ocs2::ModeSchedule::modeAtTime).
   * @param [in] eventTimes: event times of mode schedule
   * @param [in] modeSequence: modes of mode schedule (contact flags), eventTimes.size() + 1
   * @param [in] time: query time
   * @param [in] endEffectorNum: number of end effectors
   * @param [out] timesToTouchdown: time to touchdown [s], infinity if the end effector is in
   * contact at time or there is no touchdown in the schedule
   */
  inline void computeTimesToTouchdown(const std::vector<ocs2::scalar_t>& eventTimes,
    const std::vector<size_t>& modeSequence, ocs2::scalar_t time, size_t endEffectorNum,
    std::vector<ocs2::scalar_t>& timesToTouchdown)
  {
    assert(modeSequence.size() == eventTimes.size() + 1);

    constexpr ocs2::scalar_t infinity = std::numeric_limits<ocs2::scalar_t>::infinity();
    timesToTouchdown.assign(endEffectorNum, infinity);

    const size_t currentIndex = static_cast<size_t>(
      std::lower_bound(eventTimes.cbegin(), eventTimes.cend(), time) - eventTimes.cbegin());
    const legged_whole_body_control::contact_flags_t currentFlags(modeSequence[currentIndex]);

    for(size_t i = 0; i < endEffectorNum; ++i)
    {
      if(currentFlags[i]) continue;

      for(size_t k = currentIndex + 1; k < modeSequence.size(); ++k)
      {
        if(legged_whole_body_control::contact_flags_t(modeSequence[k])[i])
        {
          timesToTouchdown[i] = eventTimes[k - 1] - time;
          break;
        }
      }
    }
  }

  /**
   * Contact flags for WBC from planned (MPC policy) and measured (contact sensors) contacts:
   *  - planned stance, measured contact: contact,
   *  - planned stance, no measured contact (late touchdown, slip): no contact, so the end effector
   *    is tracked to its planned stance position (it is moved to the ground) and has no force,
   *  - planned swing, measured contact in the first half of swing: no contact, end effector
   *    is still lifting off. But when any planned stance end effector has no contact (late
   *    touchdown), it stays in contact (delayed lift off), so the robot keeps its support,
   *  - planned swing, measured contact in the second half of swing (early touchdown): contact,
   *    kept until planned stance (contact sensor can flicker after touchdown),
   *  - planned swing, no measured contact: no contact.
   * Lift off times are taken from planned contact flags given to update(), delayed lift off
   * moves them to the real lift off.
   */
  class ContactFlagsFusion
  {
    public:

      explicit ContactFlagsFusion(size_t endEffectorNum):
        endEffectorNum_(endEffectorNum),
        liftOffTimes_(endEffectorNum, 0.0) {}

      /**
       * Starts from planned flags (e.g. standing before the policy or no contact sensors),
       * end effectors in swing are assumed to lift off at time.
       */
      void reset(ocs2::scalar_t time, const legged_whole_body_control::contact_flags_t& plannedFlags)
      {
        previousPlannedFlags_ = plannedFlags;
        earlyTouchdownFlags_.reset();
        std::fill(liftOffTimes_.begin(), liftOffTimes_.end(), time);
      }

      /**
       * @param [in] time: current time
       * @param [in] plannedFlags: planned contact flags at time
       * @param [in] measuredFlags: measured contact flags
       * @param [in] timesToTouchdown: time to the next planned touchdown of every end effector
       * @return contact flags for WBC
       */
      legged_whole_body_control::contact_flags_t update(ocs2::scalar_t time,
        const legged_whole_body_control::contact_flags_t& plannedFlags,
        const legged_whole_body_control::contact_flags_t& measuredFlags,
        const std::vector<ocs2::scalar_t>& timesToTouchdown)
      {
        assert(timesToTouchdown.size() == endEffectorNum_);

        // Some planned support is missing (late touchdown)
        bool lateTouchdown = false;
        for(size_t i = 0; i < endEffectorNum_; ++i)
        {
          lateTouchdown = lateTouchdown || (plannedFlags[i] && !measuredFlags[i]);
        }

        legged_whole_body_control::contact_flags_t flags;

        for(size_t i = 0; i < endEffectorNum_; ++i)
        {
          if(plannedFlags[i])
          {
            // Planned touchdown reached, early touchdown is not needed anymore
            earlyTouchdownFlags_[i] = false;
            flags[i] = measuredFlags[i];
            continue;
          }

          if(previousPlannedFlags_[i])
          {
            liftOffTimes_[i] = time;
          }

          if(measuredFlags[i] && !earlyTouchdownFlags_[i])
          {
            // Past the middle of swing contact can only be touchdown, before it the end effector lifts off
            const bool afterMidSwing = (time - liftOffTimes_[i]) > timesToTouchdown[i];

            if(afterMidSwing)
            {
              earlyTouchdownFlags_[i] = true;
            }
            else if(lateTouchdown)
            {
              // Delayed lift off, swing starts when the support is back
              liftOffTimes_[i] = time;
              flags[i] = true;
              continue;
            }
          }

          flags[i] = earlyTouchdownFlags_[i];
        }

        previousPlannedFlags_ = plannedFlags;
        return flags;
      }

    private:

      const size_t endEffectorNum_;

      legged_whole_body_control::contact_flags_t previousPlannedFlags_;
      legged_whole_body_control::contact_flags_t earlyTouchdownFlags_;

      // Time of the last planned lift off of every end effector
      std::vector<ocs2::scalar_t> liftOffTimes_;
  };
} // namespace legged_whole_body_control_ros2

#endif
