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


#ifndef __LEGGED_PRECOMPUTATION_LEGGED_LOCOMOTION_MPC__
#define __LEGGED_PRECOMPUTATION_LEGGED_LOCOMOTION_MPC__

#include <ocs2_core/PreComputation.h>

#include <floating_base_model/FloatingBaseModelInfo.hpp>

#include <terrain_model/core/TerrainModel.hpp>

#include <legged_locomotion_mpc/common/Types.hpp>
#include <legged_locomotion_mpc/reference_manager/LeggedReferenceManager.hpp>
#include <legged_locomotion_mpc/kinematics/PinocchioForwardEndEffectorKinematicsCppAd.hpp>
#include <legged_locomotion_mpc/collision/PinocchioForwardCollisionKinematicsCppAd.hpp>
#include <legged_locomotion_mpc/torque_approx/PinocchioTorqueApproximationCppAd.hpp>
#include <legged_locomotion_mpc/weight_compensation/PinocchioWeightCompensator.hpp>

namespace legged_locomotion_mpc
{
  class LeggedPrecomputation: public ocs2::PreComputation
  {
    public:

      LeggedPrecomputation(floating_base_model::FloatingBaseModelInfo modelInfo,
        const LeggedReferenceManager& referenceManager,
        const PinocchioForwardEndEffectorKinematicsCppAd& forwardKinematics,
        const PinocchioForwardCollisionKinematicsCppAd& collisionKinematics,
        const PinocchioTorqueApproximationCppAd& torqueApproximator, 
        const PinocchioWeightCompensator& weightCompensator,
        const std::string& modelFolder = "/tmp/legged_locomotion_mpc",
        bool recompileLibraries = true,
        bool verbose = true);

      ~LeggedPrecomputation() override = default;

      LeggedPrecomputation* clone() const override;

      void request(ocs2::RequestSet request, ocs2::scalar_t t, const ocs2::vector_t &x, 
        const ocs2::vector_t &u) override;

      const vector3_t& getEndEffectorPosition(size_t endEffectorIndex) const;

      const ocs2::VectorFunctionLinearApproximation& getEndEffectorPositionDerivatives(
        size_t endEffectorIndex) const;

      const vector3_t& getEndEffectorOrientation(size_t endEffectorIndex) const;

      const ocs2::VectorFunctionLinearApproximation& getEndEffectorOrientationDerivatives(
        size_t endEffectorIndex) const;

      const vector3_t& getEndEffectorLinearVelocity(size_t endEffectorIndex) const;

      const ocs2::VectorFunctionLinearApproximation& getEndEffectorLinearVelocityDerivatives(
        size_t endEffectorIndex) const;

      const vector3_t& getEndEffectorAngularVelocity(size_t endEffectorIndex) const;

      const ocs2::VectorFunctionLinearApproximation& getEndEffectorAngularVelocityDerivatives(
        size_t endEffectorIndex) const;

      const vector3_t& getCollisionLinkPosition(size_t collisionLinkIndex) const;

      const ocs2::VectorFunctionLinearApproximation& getCollisionLinkPositionDerivatives(
        size_t collisionLinkIndex) const;

      const vector3_t& getCollisionLinkOrientation(size_t collisionLinkIndex) const;

      const ocs2::VectorFunctionLinearApproximation& getCollisionLinkOrientationDerivatives(
        size_t collisionLinkIndex) const;

      const ocs2::vector_t& getApproximatedJointTorques() const;

      const ocs2::VectorFunctionLinearApproximation& getApproximatedJointTorquesDerivatives() const;

      const matrix3_t& getRotationWorldToTerrain(size_t endEffectorIndex) const;

      const vector3_t& getSurfaceNormal(size_t endEffectorIndex) const;

      const vector3_t& getReferenceEndEffectorPosition(size_t endEffectorIndex) const;

      const vector3_t& getReferenceEndEffectorLinearVelocity(size_t endEffectorIndex) const;
      
      ocs2::scalar_t getReferenceEndEffectorTerrainClearance(size_t endEffectorIndex) const;

      const vector6_t& getEndEffectorCompensationContactWrench(size_t endEffectorIndex) const;

      /**
       * Get partial derivative w.r.t euler ZYX angles 
       * from rotation matrix and vector multiplication: d(R * v)/de.
       * Used for calculating partial derivatives of sphere positions.
       */
      matrix3_t getRotationTimesVectorGradient(const vector3_t& eulerAnglesZYX, 
        const vector3_t& vector) const;

    private:

      LeggedPrecomputation(const LeggedPrecomputation& other);

      /**
       * Updates:
       * - end effector positions 
       */
      void updateEndEffectorPositions(ocs2::scalar_t time, const ocs2::vector_t& state);
      
      /**
       * Updates:
       * - end effector orientations as euler angles
       */
      void updateEndEffectorOrientations(ocs2::scalar_t time, const ocs2::vector_t& state);
        
      /**
       * Updates:
       * - end effector linear velocities
       */
      void updateEndEffectorLinearVelocities(ocs2::scalar_t time, 
        const ocs2::vector_t& state, const ocs2::vector_t& input);
      
      /**
       * Updates:
       * - end effector angular velocities
       */
      void updateEndEffectorAngularVelocities(ocs2::scalar_t time, 
        const ocs2::vector_t& state, const ocs2::vector_t& input);

      /**
       * Updates:
       * - end effector position derivaties
       */
      void updateEndEffectorPositionDerviatives(ocs2::scalar_t time, 
        const ocs2::vector_t& state);
      
      /**
       * Updates:
       * - end effector orientation as euler angle derivatives
       */
      void updateEndEffectorOrientationDerviatives(ocs2::scalar_t time, 
        const ocs2::vector_t& state);
        
      /**
       * Updates:
       * - end effector linear velocity derivatives
       */
      void updateEndEffectorLinearVelocityDerviatives(ocs2::scalar_t time, 
        const ocs2::vector_t& state, const ocs2::vector_t& input);
      
      /**
       * Updates:
       * - end effector angular velocity derivatives
       */
      void updateEndEffectorAngularVelocityDerviatives(ocs2::scalar_t time, 
        const ocs2::vector_t& state, const ocs2::vector_t& input);
      
      /**
       * Updates:
       * - approxmiated torque values
       */
      void updateApproximatedTorquesData(ocs2::scalar_t time, const ocs2::vector_t& state, 
        const ocs2::vector_t& input);

      /**
       * Updates:
       * - approxmiated torque derivatives
       */
      void updateApproximatedTorquesDerivatives(ocs2::scalar_t time, 
        const ocs2::vector_t& state, const ocs2::vector_t& input);
      
      /**
       * Updates reference trajectory point for end effector: 
       * - position, 
       * - linear velocities
       * - terrain clearances
       * - surface normals
       * - rotation matrixes to terrain
       */
      void updateReferenceEndEffectorData(ocs2::scalar_t time);
      
      /**
       * Updates:
       * - collision links positions and orientations as euler angles
       * - collision links linear and angular velocities
       */
      void updateCollisionKinematicsData(ocs2::scalar_t time, const ocs2::vector_t& state);
      
      /**
       * Updates:
       * - collision links position and orientations as euler angle derivatives
       * - collision links linear and angular velocity derivatives
       */
      void updateCollisionKinematicsDerivatives(ocs2::scalar_t time, 
        const ocs2::vector_t& state);
      
      /**
       * Updates:
       * - end effector (3 and 6 DoF) contact forces or wrenches
       */
      void updateEndEffectorCompensationContactWrenches(ocs2::scalar_t time, 
        const ocs2::vector_t& state);

      // Cpp AD version of rotation matrix and vector multiplication: R * v
      ocs2::ad_vector_t getRotationTimesVectorCppAd(
        const Eigen::Matrix<ocs2::ad_scalar_t, 3, 1>& eulerAnglesAD, 
        const Eigen::Matrix<ocs2::ad_scalar_t, 3, 1>& vector);

      const floating_base_model::FloatingBaseModelInfo modelInfo_;
      const size_t endEffectorNumber_;
      const size_t collisionLinkNumber_;

      const LeggedReferenceManager& referenceManager_;
      const PinocchioForwardEndEffectorKinematicsCppAd forwardKinematics_;
      const PinocchioForwardCollisionKinematicsCppAd collisionKinematics_;
      const PinocchioTorqueApproximationCppAd torqueApproximator_;
      PinocchioWeightCompensator weightCompensator_;

      std::vector<vector3_t> endEffectorPositions_;
      std::vector<ocs2::VectorFunctionLinearApproximation> endEffectorPositionDerivaties_;

      std::vector<vector3_t> endEffectorEulerAngles_;
      std::vector<ocs2::VectorFunctionLinearApproximation> endEffectorEulerAngleDerivaties_;

      std::vector<vector3_t> endEffectorLinearVelocities_;
      std::vector<ocs2::VectorFunctionLinearApproximation> endEffectorLinearVelocityDerivaties_;

      std::vector<vector3_t> endEffectorAngularVelocities_;
      std::vector<ocs2::VectorFunctionLinearApproximation> endEffectorAngularVelocityDerivaties_;

      std::vector<matrix3_t> rotationWorldToTerrains_;

      std::vector<vector3_t> collisionLinkPositions_;
      std::vector<ocs2::VectorFunctionLinearApproximation> collisionLinkPositionDerivaties_;

      std::vector<vector3_t> collisionLinkEulerAngles_;
      std::vector<ocs2::VectorFunctionLinearApproximation> collisionLinkEulerAngleDerivaties_;
      
      /**
       * Includes reference trajectory in time t for:
       * positions, velocities and terrain clearances for each end effector
       */
      using EndEffectorTrajectoriesPoint = locomotion::SwingTrajectoryPlanner::EndEffectorTrajectoriesPoint;
      EndEffectorTrajectoriesPoint referenceTrajectoryPoint_;

      ocs2::vector_t torqueApproximation_;
      ocs2::VectorFunctionLinearApproximation torqueApproximationDerivatives_;

      std::vector<vector6_t> endEffectorCompensationContactWrenches_;

      /**
       * Helper Cpp AD function for getting partial derivative w.r.t euler ZYX angles
       * from rotation matrix and vector multiplication: d(R * v)/de
       */
      std::unique_ptr<ocs2::CppAdInterface> rotationMatrixVectorAdInterfacePtr_;
  };
} // namespace legged_locomotion_mpc

#endif