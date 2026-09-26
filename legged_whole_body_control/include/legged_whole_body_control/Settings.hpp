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
 * Author: Bartłomiej Krajewski (https://github.com/BartlomiejK2)
 */

#ifndef __LEGGED_WHOLE_BODY_CONTROL_TASK__
#define __LEGGED_WHOLE_BODY_CONTROL_TASK__

#include <legged_whole_body_control/Types.hpp>

namespace legged_whole_body_control 
{
  class WbcBase;
  
  struct WbcBase::FloatingBaseTrackingTaskSettings
  {
    vector3_t proportionalGain;
    vector3_t derivativeGain;
  };

  struct WbcBase::EndEffectorsTrackingTaskSettings
  {
    vector3_t proportionalGain;
    vector3_t derivativeGain;
  };

  struct WbcBase::TorqueLimitsTaskSettings
  {
    ocs2::vector_t maximumTorque;
  };

  struct WbcBase::FrictionConeTaskSettings
  {
    ocs2::scalar_t frictionCoefficient;
  };

  struct WbcBase::Settings
  {
    ocs2::scalar_t frequency;
    bool useDynamicsTask = true;
    bool useFloatingBaseTrackingTask = true;
    bool useEndEffectorsTrackingTask = true;
    bool useContactForceTrackingTask = true;
    bool useTorqueLimitsTask = true;
    bool useKinematicContactTask = true;
    bool useFrictionConeTask = true;

    FloatingBaseTrackingTaskSettings floatingBaseSettings;
    EndEffectorsTrackingTaskSettings endEffectorSettings;
    TorqueLimitsTaskSettings torqueLimitsSettings;
    FrictionConeTaskSettings frictionConeSettings;
  };
} // namespace legged_whole_body_control
#endif