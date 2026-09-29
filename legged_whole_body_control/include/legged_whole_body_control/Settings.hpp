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
  
  struct WbcBase::BaseTrackingTaskSettings
  {
    vector3_t linearFeedForwardGain;
    vector3_t linearProportionalGain;
    vector3_t linearDerivativeGain;

    vector3_t angularFeedForwardGain;
    vector3_t angularProportionalGain;
    vector3_t angularDerivativeGain;
  };

  struct WbcBase::EndEffectorsTrackingTaskSettings
  {
    // For 3 DoF end-effectors
    vector3_t linearFeedForwardGain;
    vector3_t linearProportionalGain;
    vector3_t linearDerivativeGain;

    // For 6 DoF end-effectors
    vector3_t angularFeedForwardGain;
    vector3_t angularProportionalGain;
    vector3_t angularDerivativeGain;
  };

  struct WbcBase::FrictionConeTaskSettings
  {
    ocs2::scalar_t frictionCoefficient = 0.5;
  };

  struct WbcBase::Settings
  {
    bool useDynamicsTask = true;
    bool useBaseTrackingTask = true;
    bool useEndEffectorsTrackingTask = true;
    bool useContactForceTrackingTask = true;
    bool useTorqueLimitsTask = true;
    bool useKinematicContactTask = true;
    bool useFrictionConeTask = true;

    ocs2::scalar_t weightDynamicsTask = 1.0;
    ocs2::scalar_t weightBaseTrackingTask = 1.0;
    ocs2::scalar_t weightEndEffectorsTrackingTask = 1.0;
    ocs2::scalar_t weightContactForceTrackingTask = 1.0;
    ocs2::scalar_t weightTorqueLimitsTask = 1.0;
    ocs2::scalar_t weightKinematicContactTask = 1.0;
    ocs2::scalar_t weightFrictionConeTask = 1.0;

    BaseTrackingTaskSettings baseSettings;
    EndEffectorsTrackingTaskSettings endEffectorSettings;
    FrictionConeTaskSettings frictionConeSettings;
  };
} // namespace legged_whole_body_control
#endif