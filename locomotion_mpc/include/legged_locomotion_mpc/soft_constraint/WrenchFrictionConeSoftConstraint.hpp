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

#ifndef __FORCE_WRENCH_SOFT_CONSTRAINT_LEGGED_LOCOMOTION_MPC__
#define __FORCE_WRENCH_SOFT_CONSTRAINT_LEGGED_LOCOMOTION_MPC__

#include <ocs2_core/cost/StateInputCost.h>
#include <ocs2_core/penalties/Penalties.h>

#include <floating_base_model/FloatingBaseModelInfo.hpp>

#include <legged_locomotion_mpc/common/Types.hpp>
#include <legged_locomotion_mpc/reference_manager/LeggedReferenceManager.hpp>

using namespace floating_base_model;

namespace legged_locomotion_mpc
{
  /**
   * Implements the linear wrench cone soft constraint for every 6 DoF end effector
   * h(t,x,u) >= 0: (https://hal.science/hal-02108449/document)
   */
  class WrenchFrictionConeSoftConstraint final: public ocs2::StateInputCost 
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    public:
    
      /**
      * frictionCoefficient: The coefficient of friction.
      * footLengthX: rectangular foot dimension in X axis w.r.t. foot frame 
      *  (check linked pdf for details)
      * footLengthY: rectangular foot dimension in Y axis w.r.t. foot frame 
      *  (check linked pdf for details)
      */
      struct Config 
      {
        ocs2::scalar_t frictionCoefficient = 0.7;
        ocs2::scalar_t footHalfLengthX;
        ocs2::scalar_t footHalfLengthY;
        ocs2::RelaxedBarrierPenalty::Config barrierSettings;
      };

      /**
       * Constructor
       * @param [in] referenceManager: Switched model ReferenceManager.
       * @param [in] config: Friction model settings.
       * @param [in] info: The centroidal model information.
       * @param [in] endEffectorIndex: The 6 DoF contact index.
       */
      WrenchFrictionConeSoftConstraint(const LeggedReferenceManager& referenceManager,
        Config config,
        floating_base_model::FloatingBaseModelInfo info);

      ~WrenchFrictionConeSoftConstraint() override = default;

      WrenchFrictionConeSoftConstraint* clone() const override;

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
      WrenchFrictionConeSoftConstraint(const WrenchFrictionConeSoftConstraint &other);

      Eigen::Matrix<ocs2::scalar_t, 16, 6> generateConeConstraintMatrix(
        const Config& config);

      const LeggedReferenceManager& referenceManager_;

      const Config config_;
      const floating_base_model::FloatingBaseModelInfo info_;

      Eigen::Matrix<ocs2::scalar_t, 16, 6> coneConstraintMatrix_;

      std::unique_ptr<ocs2::RelaxedBarrierPenalty> frictionBarrierPenaltyPtr_;
  };

  /**
   * Creates Wrench Friction Cone settings 
   * @param [in] filename: file path with constraint settings.
   * @param [in] fieldName: field where settings are defined
   * @param [in] verbose: verbose flag
   * @return WrenchFrictionConeSoftConstraint::Config struct
   */
  WrenchFrictionConeSoftConstraint::Config loadWrenchFrictionConeConfig(
    const std::string& filename, 
    const std::string& fieldName = "wrench_friction_cone_soft_constraint_settings",
    bool verbose = true);
} // namespace legged_locomotion_mpc

#endif