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


#ifndef __GAIT_COMMON_LEGGED_LOCOMOTION_MPC__
#define __GAIT_COMMON_LEGGED_LOCOMOTION_MPC__

#include <legged_locomotion_mpc/common/Types.hpp>

namespace legged_locomotion_mpc
{
  namespace locomotion
  {
    /**
     * Contact modes used for passing information to OCS2 framework
     * Defined as `size_t` value, where each contact flag is represented as 1 bit in position 
     * corresponding to position in contact_flags_t (0 index -> LSB)
     * 
     * End effector order in std::bitset:
     *  1. order in `threeDofEndEffectors` in `FloatingBaseModelInfo`
     *  2. order in `sixDofEndEffectors` in `FloatingBaseModelInfo`
     *
     */

    /**
     * Get contact flags from mode number
     * @param [in] modeNumber: Mode number
     * @param [in] endEffectorNumber: Number of end effectors (legs)
     * @return std::bitset with contact flags
     */
    inline contact_flags_t modeNumber2ContactFlags(const size_t modeNumber)
    {
      contact_flags_t contactFlags(modeNumber);
      return contactFlags;
    };

    /**
     * Get mode number from contact flags
     * @param [in] contactFlags: Bitset with contact flags
     * @return mode number
     */
    inline size_t contactFlags2ModeNumber(const contact_flags_t& contactFlags)
    {
      return contactFlags.to_ulong();
    };

    /**
     * Get bool contact flag from contact flags bitset
     * @param [in] contactFlags: Bitset with contact flags
     * @param [in] endEffectorIndex: Index of robots leg 
     * @return contact flag
     */
    inline bool getContactFlag(const contact_flags_t& contactFlags, size_t endEffectorIndex)
    {
      assert(endEffectorIndex < MAX_LEG_NUMBER);

      return contactFlags[endEffectorIndex];
    };

    /**
     * Normalize phase value to [0, 1.0) 
     * @param [in] value: Phase value to normalize
     * @return  Normalized phase value to [0, 1.0)
     */
    inline double normalizePhase(double value)
    {
      return value - std::floor(value);
    };

    /**
     * Get time to new end effector mode (STANCE -> SWING or SWING -> STANCE) 
     * based on end effector phase value
     * @param [in] currentPhaseState: Current normalized phase value
     * @param [in] swingRatio: Swing ratio
     * @param [in] gaitPeriod: Gait period (from dynamic gait parameters)
     * @return  time to new mode 
     */
    inline double getTimeToNextMode(double currentPhaseState, double swingRatio, double gaitPeriod)
    {
      double timeToNextMode;
      if(currentPhaseState < swingRatio)
      {
        timeToNextMode = (swingRatio - currentPhaseState) * gaitPeriod;
      }
      else
      {
        timeToNextMode = (1 - currentPhaseState) * gaitPeriod;
      }
      return timeToNextMode;
    };

    inline size_t setContactFlag(size_t currentMode, size_t endEffectorIndex, bool newState)
    {
      assert(endEffectorIndex < MAX_LEG_NUMBER);
      size_t newMode;
      if(newState)
      {
        // Set STANCE flag
        newMode = currentMode | (newState << endEffectorIndex);
      }
      else
      {
        // Set SWING flag
        newMode = currentMode & ~(!newState << endEffectorIndex);
      }
      return newMode;
    };

  } // namespace locomotion
} // namespace legged_locomotion_mpc

#endif