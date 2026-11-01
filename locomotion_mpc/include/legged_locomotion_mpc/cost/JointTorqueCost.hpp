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

#ifndef __JOINT_TORQUE_COST_LEGGED_LOCOMOTION_MPC__
#define __JOINT_TORQUE_COST_LEGGED_LOCOMOTION_MPC__

#include <pinocchio/fwd.hpp>

#include <ocs2_core/cost/StateInputCost.h>

#include <floating_base_model/FloatingBaseModelInfo.hpp>

#include <legged_locomotion_mpc/common/Types.hpp>

namespace legged_locomotion_mpc
{
  namespace cost
  {
    class JointTorqueCost: public ocs2::StateInputCost
    {
      public: 
        struct JointTorqueWeights
        {
          ocs2::vector_t weights;
        };
      
        JointTorqueCost(floating_base_model::FloatingBaseModelInfo info,
          JointTorqueWeights jointWeights);
        
        JointTorqueCost* clone() const override;
        
        ocs2::scalar_t getValue(ocs2::scalar_t time, const ocs2::vector_t& state, 
          const ocs2::vector_t& input, const ocs2::TargetTrajectories& targetTrajectories, 
          const ocs2::PreComputation& preComp) const override;
        
        ocs2::ScalarFunctionQuadraticApproximation getQuadraticApproximation(
          ocs2::scalar_t time, const ocs2::vector_t& state, const ocs2::vector_t& input,
          const ocs2::TargetTrajectories& targetTrajectories,
          const ocs2::PreComputation& preComp) const override;

      private:
        JointTorqueCost(const JointTorqueCost& rhs);

        const floating_base_model::FloatingBaseModelInfo info_;

        const JointTorqueWeights jointWeights_;
    };

  /**
   * Creates joint torque weights 
   * @param [in] filename: file path with weight settings.
   * @param [in] info: The floating base model information.
   * @param [in] fieldName: field where settings are defined
   * @param [in] verbose: verbose flag
   * @return JointTorqueWeights struct
   */
  JointTorqueCost::JointTorqueWeights loadJointTorqueWeights(const std::string& filename,
    const floating_base_model::FloatingBaseModelInfo& info, 
    const pinocchio::Model& robotModel,
    const std::string& fieldName = "joint_torque_weights",
    bool verbose = true);

  } // namespace cost
} // namespace legged_locomotion_mpc

#endif