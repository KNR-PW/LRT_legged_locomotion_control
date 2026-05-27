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

#ifndef __LEGGED_INTERFACE_LEGGED_LOCOMOTION_MPC__
#define __LEGGED_INTERFACE_LEGGED_LOCOMOTION_MPC__

#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_ddp/DDP_Settings.h>
#include <ocs2_ipm/IpmSettings.h>
#include <ocs2_sqp/SqpSettings.h>
#include <ocs2_oc/rollout/RolloutBase.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_robotic_tools/common/RobotInterface.h>

#include <floating_base_model/FloatingBaseModelInfo.hpp>

#include <legged_locomotion_mpc/common/Types.hpp>
#include <legged_locomotion_mpc/common/ModelSettings.hpp>
#include <legged_locomotion_mpc/collision/CollisionSettings.hpp>
#include <legged_locomotion_mpc/collision/PinocchoCollisionInterface.hpp>
#include <legged_locomotion_mpc/collision/PinocchioForwardCollisionKinematicsCppAd.hpp>
#include <legged_locomotion_mpc/kinematics/PinocchioForwardEndEffectorKinematicsCppAd.hpp>
#include <legged_locomotion_mpc/precomputation/LeggedPrecomputation.hpp>
#include <legged_locomotion_mpc/reference_manager/LeggedReferenceManager.hpp>
#include <legged_locomotion_mpc/initialization/LeggedInitializer.hpp>
#include <legged_locomotion_mpc/torque_approx/PinocchioTorqueApproximationCppAd.hpp>
#include <legged_locomotion_mpc/synchronization/DisturbanceSynchronizedModule.hpp>

namespace legged_locomotion_mpc
{
  class LeggedInterface: public ocs2::RobotInterface
  {
    public:

      struct Settings
      {
        // Cost
        bool useTrajectoryTrackingCost = true;
        bool useTerminalTrackingCost = true;
        bool useJointTorqueCost = true;

        // Constraints
        bool useNormalVelocityConstraint = true;

        // 3 DoF
        bool useZero3DofVelocityConstraint = true;
        bool useZeroForceConstraint = true;

        // 6 DoF
        bool useZero6DofVelocityConstraint = true;
        bool useZeroWrenchConstraint = true;

        // Soft constraints
        bool useEndEffectorPlacementSoftConstraint = true;
        bool useJointLimitsSoftConstraint = true;
        bool useJointTorqueLimitsSoftConstraint = true;
        bool useTerrainAvoidanceSoftConstraint = true;
        bool useSelfCollisionAvoidanceSoftConstraint = true;

        // 3 DoF
        bool useForceFrictionConeSoftConstraint = true;
        
        // 6 DoF
        bool useWrenchFrictionConeSoftConstraint = true;
        bool useNormalOrientationSoftConstraint = true;
      };

      LeggedInterface(ocs2::scalar_t initTime, const ocs2::vector_t& currentState, 
        std::unique_ptr<terrain_model::TerrainModel> currentTerrainModel, 
        const std::string& taskFile, const std::string& modelFile,
        const std::string& urdfFile);

      /**
       * Gets the ReferenceManager.
       * @return a shared pointer to the ReferenceManager.
       */
      std::shared_ptr<ocs2::ReferenceManagerInterface> getReferenceManagerPtr() const;

      /**
       * Gets the vector of SynchronizedModule.
       * @return a vector of shared pointers to the SynchronizedModules.
       */
      std::vector<std::shared_ptr<ocs2::SolverSynchronizedModule>> getSynchronizedModulePtrs() const;

      /**
       * Gets the LeggedReferenceManager.
       * @return a reference to the LeggedReferenceManager.
       */
      LeggedReferenceManager& getLeggedReferenceManager();
      
      /**
       * Gets the Rollout.
       * @return a reference to the RolloutBase.
       */
      ocs2::RolloutBase& getRollout();

      /**
       * @brief Get the optimal control problem definition
       * @return reference to the problem object
       */
      const ocs2::OptimalControlProblem& getOptimalControlProblem() const;

      /**
       * @brief getInitializer
       * @return reference to the internal solver initializer
       */
      const ocs2::Initializer& getInitializer() const;
      
      /**
       * @brief Get DDP settings
       * @return const reference to the DDP settings
       */
      const ocs2::ddp::Settings& ddpSettings() const;
      
      /**
       * @brief Get MPC settings
       * @return const reference to the MPC settings
       */
      const ocs2::mpc::Settings& mpcSettings() const;
      
      /**
       * @brief Get rollout settings
       * @return const reference to the rollout settings
       */
      const ocs2::rollout::Settings& rolloutSettings() const;
      
      /**
       * @brief Get SQP settings
       * @return const reference to the SQP settings
       */
      const ocs2::sqp::Settings& sqpSettings() const;
      
      /**
       * @brief Get IPM settings
       * @return const reference to the IPM settings
       */
      const ocs2::ipm::Settings& ipmSettings() const;

      /**
       * @brief Get Interface settings
       * @return const reference to the Interface settings
       */
      const Settings& interfaceSettings() const;

      /**
       * @brief Get Legged model settings
       * @return const reference to the Legged model settings
       */
      const ModelSettings& modelSettings() const;

      /**
       * @brief Get Collision model settings
       * @return const reference to the Collision model settings
       */
      const collision::CollisionSettings& collisionSettings() const;

      /**
       * @brief Get Floating base model info
       * @return const reference to the Floating base model info
       */
      const floating_base_model::FloatingBaseModelInfo& floatingBaseModelInfo() const;

      /**
       * @brief Get PinocchioInterface
       * @return const reference to the PinocchioInterface
       */
      const ocs2::PinocchioInterface& pinocchioInterface() const;
      
      /**
       * @brief Get end effector forward kinematics
       * @return const reference to the end effector forward kinematics
       */
      const PinocchioForwardEndEffectorKinematicsCppAd& forwardKinematics() const;
      
      /**
       * @brief Get collision forward kinematics
       * @return const reference to the collision forward kinematics
       */
      const PinocchioForwardCollisionKinematicsCppAd& forwardCollisionKInematics() const;
      
      /**
       * @brief Get collision interface
       * @return const reference to the collision interface
       */
      const collision::PinocchioCollisionInterface& collisionInterface() const;
      
      /**
       * @brief Get torque approximator
       * @return const reference to the torque approximator
       */
      const PinocchioTorqueApproximationCppAd& torqueApproximator() const;
      
      /**
       * @brief Get weight compensator
       * @return reference to the weight compensator
       */
      PinocchioWeightCompensator& weightCompensator();

      /**
       * @brief Get disturbance wrench synchronized module
       * @return reference to disturbance wrench synchronized module
       */
      synchronization::DisturbanceSynchronizedModule& disturbanceModule();

      /**
       * @brief Get initial system state
       * @return const reference to the initial system state
       */
      const ocs2::vector_t getInitialState() const;

    private:

      void createHelperClasses();

      void createReferenceManager(ocs2::scalar_t initTime, 
        const ocs2::vector_t& currentState, 
        std::unique_ptr<terrain_model::TerrainModel> currentTerrainModel, 
        const std::string& modelFile, const std::string& urdfFile);

      void createInitializer();

      void createOptimalProblem(ocs2::scalar_t initTime, 
        const ocs2::vector_t& currentState, const std::string& modelFile);

      void creatSynchronizedModule();

      void createRollout();
      
      Settings interfaceSettings_;
      ModelSettings modelSettings_;
      collision::CollisionSettings collisionSettings_;
      floating_base_model::FloatingBaseModelInfo floatingBaseModelInfo_;

      ocs2::ddp::Settings ddpSettings_;
      ocs2::mpc::Settings mpcSettings_;
      ocs2::sqp::Settings sqpSettings_;
      ocs2::ipm::Settings ipmSettings_;

      ocs2::OptimalControlProblem optimalProblem_;
      std::unique_ptr<ocs2::Initializer> initializerPtr_;
      std::shared_ptr<LeggedReferenceManager> referenceManagerPtr_;
      
      ocs2::rollout::Settings rolloutSettings_;
      std::unique_ptr<ocs2::RolloutBase> rolloutPtr_;

      std::unique_ptr<ocs2::PinocchioInterface> pinocchioInterfacePtr_;
      
      std::shared_ptr<ocs2::SolverSynchronizedModule> disturbanceModulePtr_;
      
      std::unique_ptr<PinocchioForwardEndEffectorKinematicsCppAd> endEffectorForwardKinematics_;
      std::unique_ptr<PinocchioForwardCollisionKinematicsCppAd> collisionForwardKinematics_;
      std::unique_ptr<collision::PinocchioCollisionInterface> collisionInterface_;
      std::unique_ptr<PinocchioTorqueApproximationCppAd> torqueApproximator_;
      std::unique_ptr<PinocchioWeightCompensator> weightCompensator_;

      ocs2::vector_t initialState_;
  };

  /**
   * Creates Legged interface settings 
   * @param [in] filename: file path with model settings.
   * @param [in] fieldName: field where settings are defined
   * @param [in] verbose: verbose flag
   * @return LeggedInterface::Settings struct
   */
  LeggedInterface::Settings loadLeggedInterfaceSettings(const std::string& filename,
    const std::string& fieldName = "legged_interface_settings",
    bool verbose = "true");
} // namespace legged_locomotion_mpc
#endif