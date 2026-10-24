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

#ifndef __END_EFFECTOR_PLACEMENT_SOFT_CONSTRAINT_LEGGED_LOCOMOTION_MPC__
#define __END_EFFECTOR_PLACEMENT_SOFT_CONSTRAINT_LEGGED_LOCOMOTION_MPC__

#include <ocs2_core/cost/StateCost.h>
#include <ocs2_core/penalties/Penalties.h>

#include <floating_base_model/FloatingBaseModelInfo.hpp>

#include <legged_locomotion_mpc/common/Types.hpp>
#include <legged_locomotion_mpc/common/ModelSettings.hpp>
#include <legged_locomotion_mpc/reference_manager/LeggedReferenceManager.hpp>

namespace legged_locomotion_mpc
{
  /**
   * Constraint that ensures that end effectors (feet) are inside terrain, that
   * can be safely stepped on. 
   */
  class EndEffectorPlacementSoftConstraint final: public ocs2::StateCost 
  {
    public:

      struct Settings
      {
        // Safety radius for each foot (representing contact pressure circle)
        std::vector<ocs2::scalar_t> endEffectorSafetyRadiuses;

        ocs2::RelaxedBarrierPenalty::Config barrierSettings;
      };

      /**
       * Constructor
       * @param [in] info: Floating Base model info.
       * @param [in] referenceManager: Legged Reference Manager
       * @param [in] endEffectorRadiuses: Safety radius for each end effector.
       * @param [in] settings: Relaxed barrier penalty settings 
       */
      EndEffectorPlacementSoftConstraint(
        floating_base_model::FloatingBaseModelInfo info,
        const LeggedReferenceManager& referenceManager,
        Settings settings);

      ~EndEffectorPlacementSoftConstraint() override = default;

      EndEffectorPlacementSoftConstraint* clone() const override;

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
      EndEffectorPlacementSoftConstraint(const EndEffectorPlacementSoftConstraint &rhs);
      
      const LeggedReferenceManager& referenceManager_;
      const size_t endEffectorNum_;
        
      const floating_base_model::FloatingBaseModelInfo info_;
      const std::vector<ocs2::scalar_t> endEffectorRadiuses_;

      std::unique_ptr<ocs2::RelaxedBarrierPenalty> placementRelaxedBarrierPenaltyPtr_;
    };

    /**
     * Creates EndEffectorPlacementSoftConstraint settings 
     * @param [in] filename: file path with model settings.
     * @param [in] fieldName: field where settings are defined
     * @param [in] modelSettings: model settings
     * @param [in] verbose: verbose flag
     * @return EndEffectorPlacementSoftConstraint::Settings struct
     */
    EndEffectorPlacementSoftConstraint::Settings loadEndEffectorPlacementSoftConstraintSettings(
      const std::string& filename,
      const ModelSettings modelSettings,
      const std::string& fieldName = "end_effector_soft_constraint_settings",
      bool verbose = true);

} // namespace legged_locomotion_mpc

#endif