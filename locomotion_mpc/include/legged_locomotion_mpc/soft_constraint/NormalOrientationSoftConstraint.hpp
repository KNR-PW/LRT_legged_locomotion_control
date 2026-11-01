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

#ifndef __NORMAL_ORIENTATION_CONSTRAINT_LEGGED_LOCOMOTION_MPC__
#define __NORMAL_ORIENTATION_CONSTRAINT_LEGGED_LOCOMOTION_MPC__

#include <ocs2_core/cost/StateCost.h>
#include <ocs2_core/penalties/Penalties.h>

#include <floating_base_model/FloatingBaseModelInfo.hpp>

#include <legged_locomotion_mpc/common/Types.hpp>
#include <legged_locomotion_mpc/reference_manager/LeggedReferenceManager.hpp>

using namespace floating_base_model;

namespace legged_locomotion_mpc
{
  /**
   * Constraint that ensures that every 6DoF end effector's z axis is in parallel with 
   * terrain normal vector
   */
  class NormalOrientationSoftConstraint final: public ocs2::StateCost
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    public:
    
      struct Settings
      {
        ocs2::scalar_t barrierMu;
      };

      /**
       * Constructor
       * @param [in] referenceManager: Switched model ReferenceManager.
       * @param [in] settings: Soft constraint settings.
       * @param [in] info: The centroidal model information.
       */
      NormalOrientationSoftConstraint(const LeggedReferenceManager& referenceManager,
        Settings settings, floating_base_model::FloatingBaseModelInfo info);

      ~NormalOrientationSoftConstraint() override = default;

      NormalOrientationSoftConstraint* clone() const override;

      /** Get cost term value */
      ocs2::scalar_t getValue(ocs2::scalar_t time, const ocs2::vector_t& state, 
        const ocs2::TargetTrajectories& targetTrajectories, 
        const ocs2::PreComputation& preComp) const override;
        
      /** Get cost term quadratic approximation */
      ocs2::ScalarFunctionQuadraticApproximation getQuadraticApproximation(
        ocs2::scalar_t time, const ocs2::vector_t& state, 
        const ocs2::TargetTrajectories& targetTrajectories,
        const ocs2::PreComputation& preComp) const override;

    private:
      NormalOrientationSoftConstraint(const NormalOrientationSoftConstraint &other);

      const LeggedReferenceManager& referenceManager_;

      const floating_base_model::FloatingBaseModelInfo info_;

      std::unique_ptr<ocs2::QuadraticPenalty> normalRelaxedBarrierPenaltyPtr_;
  };

  /**
   * Creates NormalOrientationSoftConstraint settings 
   * @param [in] filename: file path with constraint settings.
   * @param [in] fieldName: field where settings are defined
   * @param [in] verbose: verbose flag
   * @return NormalOrientationSoftConstraint::Settings struct
   */
  NormalOrientationSoftConstraint::Settings loadNormalOrientationSoftConstraintSettings(
    const std::string& filename, 
    const std::string& fieldName = "normal_orientation_soft_constraint_settings",
    bool verbose = true);
} // namespace legged_locomotion_mpc

#endif