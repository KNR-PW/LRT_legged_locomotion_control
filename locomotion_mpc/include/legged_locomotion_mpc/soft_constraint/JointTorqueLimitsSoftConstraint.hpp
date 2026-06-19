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
 * Based on: rgrandia (https://github.com/leggedrobotics/ocs2)
 */


#ifndef __TORQUE_LIMITS_SOFT_CONSTRAINT_LEGGED_LOCOMOTION_MPC__
#define __TORQUE_LIMITS_SOFT_CONSTRAINT_LEGGED_LOCOMOTION_MPC__


#include <ocs2_core/cost/StateInputCost.h>
#include <ocs2_core/penalties/Penalties.h>

#include <floating_base_model/FloatingBaseModelInfo.hpp>

#include <legged_locomotion_mpc/common/Types.hpp>
#include <legged_locomotion_mpc/torque_approx/PinocchioTorqueApproximationCppAd.hpp>


namespace legged_locomotion_mpc
{
  class JointTorqueLimitsSoftConstraint final: public ocs2::StateInputCost
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      struct Settings
      {
        ocs2::RelaxedBarrierPenalty::Config barrierSettings;
      };
    
      JointTorqueLimitsSoftConstraint(floating_base_model::FloatingBaseModelInfo info,
        ocs2::vector_t nominalTorques, ocs2::vector_t torqueLimits, Settings settings);

      ~JointTorqueLimitsSoftConstraint() override = default;

      JointTorqueLimitsSoftConstraint* clone() const override;

      /** Get cost term value */
      ocs2::scalar_t getValue(ocs2::scalar_t time, const ocs2::vector_t& state,
        const ocs2::vector_t& input, const ocs2::TargetTrajectories& targetTrajectories,
        const ocs2::PreComputation& preComp) const override;
        
      /** Get cost term quadratic approximation */
      ocs2::ScalarFunctionQuadraticApproximation getQuadraticApproximation(
        ocs2::scalar_t time, const ocs2::vector_t& state, const ocs2::vector_t& input,
        const ocs2::TargetTrajectories& targetTrajectories,
        const ocs2::PreComputation& preComp) const override;

    private:

      JointTorqueLimitsSoftConstraint(const JointTorqueLimitsSoftConstraint &rhs);

      const floating_base_model::FloatingBaseModelInfo info_;
        
      std::unique_ptr<ocs2::RelaxedBarrierPenalty> torqueRelaxedBarrierPenaltyPtr_;

      const ocs2::vector_t torqueLimits_;
      
      ocs2::scalar_t nominalTorqueOffset_;
    };

    /**
     * Creates JointTorqueLimitsSoftConstraint settings 
     * @param [in] filename: file path with model settings.
     * @param [in] fieldName: field where settings are defined
     * @param [in] verbose: verbose flag
     * @return JointTorqueLimitsSoftConstraint::Settings struct
     */
    JointTorqueLimitsSoftConstraint::Settings loadJointTorqueLimitsSoftConstraintSettings(
      const std::string& filename,
      const std::string& fieldName = "joint_torque_limits_soft_constraint_settings",
      bool verbose = "true");
} // namespace legged_locomotion_mpc

#endif