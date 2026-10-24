/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

Modified by Bartłomiej Krajewski (https://github.com/BartlomiejK2), 2025

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#ifndef __FORCE_FRICTION_SOFT_CONSTRAINT_LEGGED_LOCOMOTION_MPC__
#define __FORCE_FRICTION_SOFT_CONSTRAINT_LEGGED_LOCOMOTION_MPC__

#include <ocs2_core/cost/StateInputCost.h>
#include <ocs2_core/penalties/Penalties.h>

#include <floating_base_model/FloatingBaseModelInfo.hpp>

#include <legged_locomotion_mpc/common/Types.hpp>
#include <legged_locomotion_mpc/reference_manager/LeggedReferenceManager.hpp>

namespace legged_locomotion_mpc
{
  /**
   * Implements the soft constraint h(t,x,u) >= 0
   *
   * For every 3 DoF end effector:
   * frictionCoefficient * Fz - sqrt(Fx * Fx + Fy * Fy + regularization) >= 0
   *
   * The regularization prevents the constraint gradient / hessian to go to infinity when Fx = Fz = 0. It also creates a parabolic safety
   * margin to the friction cone. For example: when Fx = Fy = 0 the constraint zero-crossing will be at Fz = 1/frictionCoefficient *
   * sqrt(regularization) instead of Fz = 0
   *
   */
  class ForceFrictionConeSoftConstraint final: public ocs2::StateInputCost
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    public:
    
      /**
      * frictionCoefficient: The coefficient of friction.
      * regularization: A positive number to regulize the friction constraint. refer to the FrictionConeConstraint documentation.
      * gripperForce: Gripper force in normal direction.
      * hessianDiagonalShift: The Hessian shift to assure a strictly-convex quadratic constraint approximation.
      * barrierSettings: relaxed barrier settings
      */
      struct Config 
      {
        ocs2::scalar_t frictionCoefficient = 0.7;
        ocs2::scalar_t regularization = 25.0;
        ocs2::scalar_t hessianDiagonalShift = 1e-6;
        ocs2::RelaxedBarrierPenalty::Config barrierSettings;
      };

      /**
       * Constructor
       * @param [in] referenceManager: Legged model ReferenceManager.
       * @param [in] config: Friction model settings.
       * @param [in] info: The floating base model information.
       */
      ForceFrictionConeSoftConstraint(
        const LeggedReferenceManager& referenceManager,
        Config config,
        floating_base_model::FloatingBaseModelInfo info);

      ~ForceFrictionConeSoftConstraint() override = default;

      ForceFrictionConeSoftConstraint* clone() const override;

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
      
      ForceFrictionConeSoftConstraint(const ForceFrictionConeSoftConstraint &other);

      ocs2::scalar_t coneConstraint(const vector3_t &localForce) const;

      const LeggedReferenceManager& referenceManager_;

      const Config config_;
      const floating_base_model::FloatingBaseModelInfo info_;

      std::unique_ptr<ocs2::RelaxedBarrierPenalty> frictionBarrierPenaltyPtr_;
  };

  /**
   * Creates Force Friction Cone settings 
   * @param [in] filename: file path with constraint settings.
   * @param [in] fieldName: field where settings are defined
   * @param [in] verbose: verbose flag
   * @return ForceFrictionConeSoftConstraint::Config struct
   */
  ForceFrictionConeSoftConstraint::Config loadForceFrictionConeConfig(const std::string& filename,
    const std::string& fieldName = "force_friction_cone_soft_constraint_settings",
    bool verbose = true);
} // namespace legged_locomotion_mpc

#endif