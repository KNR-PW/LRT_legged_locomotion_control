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

#ifndef __TRAJECTORY_TRACKING_COST_LEGGED_LOCOMOTION_MPC__
#define __TRAJECTORY_TRACKING_COST_LEGGED_LOCOMOTION_MPC__

#include <ocs2_core/cost/StateInputCost.h>

#include <floating_base_model/FloatingBaseModelInfo.hpp>

#include <legged_locomotion_mpc/common/Types.hpp>
#include <legged_locomotion_mpc/common/ModelSettings.hpp>
#include <legged_locomotion_mpc/common/Utils.hpp>
#include <legged_locomotion_mpc/reference_manager/LeggedReferenceManager.hpp>

namespace legged_locomotion_mpc
{
  namespace cost
  {
    class TrajectoryTrackingCost: public ocs2::StateInputCost
    {
      public: 
        struct BaseWeights
        {
          vector3_t position;
          vector3_t rotation;
          vector3_t linearVelocity;
          vector3_t angularVelocity;
        };
      
        struct JointWeights
        {
          ocs2::vector_t positions;
          ocs2::vector_t velocities;
        };
      
        struct EndEffectorWeights
        {
          std::vector<vector3_t> positions;
          std::vector<vector3_t> velocities; // linear
          std::vector<vector3_t> forces;
          std::vector<vector3_t> torques; // only for 6DoF contacts
        };
      
        TrajectoryTrackingCost(floating_base_model::FloatingBaseModelInfo info,
          BaseWeights baseWeights, JointWeights jointWeights,
          EndEffectorWeights endEffectorWeights, 
          const std::string& modelFolder = "/tmp/legged_locomotion_mpc",
          bool recompileLibraries = true,
          bool verbose = true);
        
        TrajectoryTrackingCost* clone() const override;
        
        ocs2::scalar_t getValue(ocs2::scalar_t time, const ocs2::vector_t& state, 
          const ocs2::vector_t& input, const ocs2::TargetTrajectories& targetTrajectories, 
          const ocs2::PreComputation& preComp) const override;
        
        ocs2::ScalarFunctionQuadraticApproximation getQuadraticApproximation(
          ocs2::scalar_t time, const ocs2::vector_t& state, const ocs2::vector_t& input,
          const ocs2::TargetTrajectories& targetTrajectories,
          const ocs2::PreComputation& preComp) const override;

      private:
        TrajectoryTrackingCost(const TrajectoryTrackingCost& rhs);

        /**
         * Cpp AD version of log3 of tagret (parameter) and 
         * actual euler angles (differentiable values)
         * */ 
        ocs2::ad_vector_t getLog3CppAd(
          const Eigen::Matrix<ocs2::ad_scalar_t, 3, 1>& currentEulerAngles, 
          const Eigen::Matrix<ocs2::ad_scalar_t, 3, 1>& targetEulerAngles);

        const floating_base_model::FloatingBaseModelInfo info_;

        const BaseWeights baseWeights_;
        const JointWeights jointWeights_;
        const EndEffectorWeights endEffectorWeights_;

        /**
         * Helper Cpp AD function for getting partial derivative w.r.t euler ZYX angles
         * from log3 of tagret (parameter) and actual euler angles (differentiable values):
         * dlog3(e)/de
         */
        std::unique_ptr<ocs2::CppAdInterface> log3AdInterfacePtr_;
    };

  /**
   * Creates base weights 
   * @param [in] filename: file path with weight settings.
   * @param [in] fieldName: field where settings are defined
   * @param [in] verbose: verbose flag
   * @return BaseWeights struct
   */
  TrajectoryTrackingCost::BaseWeights loadBaseWeights(const std::string& filename,
    const std::string& fieldName = "base_weights",
    bool verbose = true);

  /**
   * Creates joint weights 
   * @param [in] filename: file path with weight settings.
   * @param [in] info: The floating base model information.
   * @param [in] robotModel: pinocchio robot model from pinocchioInterface
   * @param [in] fieldName: field where settings are defined
   * @param [in] verbose: verbose flag
   * @return JointWeights struct
   */
  TrajectoryTrackingCost::JointWeights loadJointWeights(const std::string& filename,
    const floating_base_model::FloatingBaseModelInfo& info,
    const pinocchio::Model& robotModel,
    const std::string& fieldName = "joint_weights",
    bool verbose = true);

  /**
   * Creates end effector weights 
   * @param [in] filename: file path with weight settings.
   * @param [in] fieldName: field where settings are defined
   * @param [in] verbose: verbose flag
   * @return EndEffectorWeights struct
   */
  TrajectoryTrackingCost::EndEffectorWeights loadEndEffectorWeights(
    const std::string& filename,
    const ModelSettings& modelSettings,
    const std::string& fieldName = "end_effector_weights",
    bool verbose = true);

  } // namespace cost
} // namespace legged_locomotion_mpc

#endif
