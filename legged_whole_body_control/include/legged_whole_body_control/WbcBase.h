//
// Created by qiayuan on 2022/7/1.
//

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
 * Modified by: Bartłomiej Krajewski (https://github.com/BartlomiejK2)
 */

#ifndef __LEGGED_WHOLE_BODY_CONTROL_WBC_BASE__
#define __LEGGED_WHOLE_BODY_CONTROL_WBC_BASE__

#include <floating_base_model/FloatingBaseModelInfo.hpp>

#include <legged_whole_body_control/Settings.hpp>
#include <legged_whole_body_control/Task.h>

namespace legged_whole_body_control 
{
  // Decision Variables: x = [\dot u^T, F^T, \tau^T]^T
  class WbcBase 
  {
    public:

      WbcBase(const ocs2::PinocchioInterface& pinocchioInterface, 
        floating_base_model::FloatingBaseModelInfo info, 
        Settings settings);

      void updateDesired(ocs2::scalar_t time, const ocs2::vector_t& state, 
        const ocs2::vector_t& input);

      void updateCurrent(ocs2::scalar_t time, const ocs2::vector_t& state, 
        const ocs2::vector_t& input);

      void updateContactFlags(ocs2::scalar_t time, const contact_flags_t& contactFlags);

      vector6_t& getBaseAcceleration() const;

      vector_t getJointAcceleration() const;

      vector_t getJointTorque() const;

      vector3_t getEndEffectorForce(size_t endEffectorIndex) const;

      vector6_t getEndEffectorWrench(size_t getEndEffectorIndex) const;

      virtual vector_t update(ocs2::scalar_t time);

   protected:

    void updateMeasured(const vector_t& rbdStateMeasured);

    void updateDesired(const vector_t& stateDesired, const vector_t& inputDesired);

    size_t getNumDecisionVars() const { return numDecisionVars_; }

    Task formulateFloatingBaseEomTask();
    Task formulateTorqueLimitsTask();
    Task formulateNoContactMotionTask();
    Task formulateFrictionConeTask();
    Task formulateBaseAccelTask(const vector_t& stateDesired, 
      const vector_t& inputDesired, scalar_t period);
    Task formulateSwingLegTask();
    Task formulateContactForceTask(const vector_t& inputDesired) const;

    size_t numberOfDecisionVariables_;
    
    Settings settings_;
    floating_base_model::FloatingBaseModelInfo info_;

    ocs2::PinocchioInterface pinocchioInterfaceMeasured_;
    ocs2::PinocchioInterface pinocchioInterfaceDesired_;

    ocs2::vector_t stateMeasured_;
    ocs2::vector_t inputMeasured_;

    ocs2::vector_t stateDesired_;
    ocs2::vector_t inputDesired_;

    ocs2::matrix_t stackedJacobians_;
    ocs2::matrix_t stackedJacobianDerivatives_;

    contact_flags_t contactFlag_;
  };
} // namespace legged_whole_body_control
#endif