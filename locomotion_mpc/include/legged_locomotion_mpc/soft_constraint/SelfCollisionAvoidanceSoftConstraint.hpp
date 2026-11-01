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

#ifndef __SELF_COLLISION_AVOIDANCE_SOFT_CONSTRAINT_LEGGED_LOCOMOTION_MPC__
#define __SELF_COLLISION_AVOIDANCE_SOFT_CONSTRAINT_LEGGED_LOCOMOTION_MPC__

#include <ocs2_core/cost/StateCost.h>
#include <ocs2_core/penalties/Penalties.h>

#include <floating_base_model/FloatingBaseModelInfo.hpp>

#include <legged_locomotion_mpc/common/Types.hpp>
#include <legged_locomotion_mpc/collision/PinocchoCollisionInterface.hpp>
#include <legged_locomotion_mpc/reference_manager/LeggedReferenceManager.hpp>

namespace legged_locomotion_mpc
{
  /**
   * Constraint that ensures that end effectors and/or links do not collide with each other.
   * min(dist(p_1 - p_2)) - r_1 - r_2 + relax > 0 where:
   * min(dist(p_1 - p_2)) - minimal distance between approximation 
   * spheres of two end effector or links
   * r_1, r_2 - radiuses of end effector or collision link sphere
   * relax - relaxation constraint value
   */
  class SelfCollisionAvoidanceSoftConstraint final: public ocs2::StateCost 
  {
    public:

      struct Settings
      {
        ocs2::RelaxedBarrierPenalty::Config barrierSettings;
      };

      /**
       * Constructor
       * @param [in] info: Floating Base model info.
       * @param [in] sphereInterface: interface for sphere approximation
       * @param [in] collisionPairIndices: Pair of self collsion 
       * indexes of end effectors (0 : endEffectorNum - 1)
       * or collision links (endEffectorNum : endEffectorNum + collisionNum - 1).
       * @param [in] referenceManager: Legged Reference Manager
       * @param [in] relaxations: Relax constraint values (for all pairs).
       * @param [in] settings: self collision avoidance soft constraint internal settings
       */
      SelfCollisionAvoidanceSoftConstraint(
        floating_base_model::FloatingBaseModelInfo info,
        const collision::CollisionSettings& collisionSettings,
        const collision::PinocchioCollisionInterface& collisionInterface,
        const LeggedReferenceManager& referenceManager,
        Settings settings);

      ~SelfCollisionAvoidanceSoftConstraint() override = default;

      SelfCollisionAvoidanceSoftConstraint* clone() const override;

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
      SelfCollisionAvoidanceSoftConstraint(const SelfCollisionAvoidanceSoftConstraint& rhs);
      
      const size_t threeDofEndEffectorNum_;
      const size_t sixDofEndEffectorNum_;
      const size_t endEffectorNum_;

      // 3 DoF End effector - 3 DoF End effector
      std::vector<std::pair<size_t, size_t>> endEffector33DoFPairIndices_;
      std::vector<ocs2::scalar_t> endEffector33DoFPairRelaxations_;
      // 3 DoF End effector - 6 DoF End effector
      std::vector<std::pair<size_t, size_t>> endEffector36DoFPairIndices_;
      std::vector<ocs2::scalar_t> endEffector36DoFPairRelaxations_;
      // 6 DoF End effector - 6 DoF End effector
      std::vector<std::pair<size_t, size_t>> endEffector66DoFPairIndices_;
      std::vector<ocs2::scalar_t> endEffector66DoFPairRelaxations_;
      // 3 DoF End effector - Collision link
      std::vector<std::pair<size_t, size_t>> endEffector3DoFLinkIndices_;
      std::vector<ocs2::scalar_t> endEffector3DoFLinkRelaxations_;
      // 6 DoF End effector - Collision link
      std::vector<std::pair<size_t, size_t>> endEffector6DoFLinkIndices_;
      std::vector<ocs2::scalar_t> endEffector6DoFLinkRelaxations_;
      // Collision link - Collision link
      std::vector<std::pair<size_t, size_t>> collisionLinkPairIndices_;
      std::vector<ocs2::scalar_t> collisionLinkPairRelaxations_;

      const LeggedReferenceManager& referenceManager_;
      const collision::PinocchioCollisionInterface& collisionInterface_;
      
      std::unique_ptr<ocs2::RelaxedBarrierPenalty> selfAvoidancePenaltyPtr_;

      // Buffer for sphere positions in world frame (every solver thread has own clone)
      mutable std::vector<vector3_t> sphereWorldPositions_;
  };

  /**
   * Creates SelfCollisionAvoidanceSoftConstraint settings 
   * @param [in] filename: file path with model settings.
   * @param [in] fieldName: field where settings are defined
   * @param [in] verbose: verbose flag
   * @return SelfCollisionAvoidanceSoftConstraint::Settings struct
   */
  SelfCollisionAvoidanceSoftConstraint::Settings loadSelfCollisionAvoidanceSoftConstraintSettings(
    const std::string& filename,
    const std::string& fieldName = "self_collison_avoidance_soft_constraint_settings",
    bool verbose = true);
} // namespace legged_locomotion_mpc

#endif