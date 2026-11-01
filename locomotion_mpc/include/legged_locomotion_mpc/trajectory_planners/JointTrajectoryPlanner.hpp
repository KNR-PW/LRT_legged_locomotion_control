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


#ifndef __JOINT_TRAJECTORY_PLANNER_LEGGED_LOCOMOTION_MPC__
#define __JOINT_TRAJECTORY_PLANNER_LEGGED_LOCOMOTION_MPC__

#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_mpc/SystemObservation.h>

#include <multi_end_effector_kinematics/MultiEndEffectorKinematics.hpp>

#include <floating_base_model/FloatingBaseModelInfo.hpp>

#include <legged_locomotion_mpc/common/Types.hpp>
#include <legged_locomotion_mpc/locomotion/SwingTrajectoryPlanner.hpp>

namespace legged_locomotion_mpc
{
  namespace planners
  {
    class JointTrajectoryPlanner
    {
      public:

        JointTrajectoryPlanner(floating_base_model::FloatingBaseModelInfo modelInfo,
          multi_end_effector_kinematics::MultiEndEffectorKinematics&& kinematicsSolver);
        
        using position_trajectories = std::vector<std::vector<vector3_t>>;
        using velocity_trajectories = std::vector<std::vector<vector3_t>>;
        using normal_trajectories = std::vector<std::vector<vector3_t>>;

        /** 
         * Update target trajectories with force and wrench trajectory.
         * Call BaseTrajectoryPlanner::updateTrajectory() before calling this function.
         * @param [in] currentState: current robot state
         * @param [in] endEffectorPositionTrajectories: trajectory of end effector positions
         * @param [in] endEffectorVelocityTrajectories: trajectory of end effector velocities
         * @param [in] endEffectorNormalTrajectories: trajectory of end effector normals
         * @param [out] targetTrajectories: target trajectories
         * 
         * @remark Every element of endEffectorPositionTrajectories 
         * and endEffectorVelocityTrajectories should be generated at every 
         * time point in targetTrajectories.timeTrajectory 
         * (endEffectorPositionTrajectories[i] at targetTrajectories.timeTrajectory[i]).
         * Generate contact trajectory with 
         * SwingTrajectoryPlanner::getEndEffectorPositionTrajectories()
         * and SwingTrajectoryPlanner::getEndEffectorVelocityTrajectories()
         * using targetTrajectories.timeTrajectory.
         */
        void updateTrajectory(const ocs2::SystemObservation& currentObservation,
          ocs2::TargetTrajectories& targetTrajectories, 
          const position_trajectories& endEffectorPositionTrajectories,
          const velocity_trajectories& endEffectorVelocityTrajectories,
          const normal_trajectories& endEffectorNormalTrajectories);

        ocs2::vector_t computeJointPositions(const ocs2::vector_t& actualJointPositions, 
          const vector6_t& basePose, 
          const std::vector<vector3_t>& endEffectorPositions, 
          const std::vector<vector3_t>& endEffectorNormals);

        /**
         * @param [in] previousJointVelocities: returned when solver does not return
         * joint velocities
         */
        ocs2::vector_t computeJointVelocities(const ocs2::vector_t& actualJointPositions,
          const vector6_t& basePose, const vector6_t& baseVelocity,
          const std::vector<vector3_t>& endEffectorVelocities,
          const ocs2::vector_t& previousJointVelocities);
    
      private:
  
        floating_base_model::FloatingBaseModelInfo modelInfo_;

        multi_end_effector_kinematics::MultiEndEffectorKinematics kinematicsSolver_;

        const ocs2::vector_t jointPositionUpperLimits_; 
        const ocs2::vector_t jointPositionLowerLimits_;
        const ocs2::vector_t jointVelocityLimits_;
    };
    
     /**
     * Creates KinematicsModel settings 
     * @param [in] filename: file path with joint planner settings.
     * @param [in] fieldName: field where settings are defined
     * @param [in] verbose: verbose flag
     * @return multi_end_effector_kinematics::KinematicsModelSettings struct
     */
    multi_end_effector_kinematics::KinematicsModelSettings loadKinematicsModelSettings(
      const std::string& filename, 
      const std::string& fieldName = "legged_model_settings", 
      bool verbose = true);
    
     /**
     * Creates InverseSolver settings 
     * @param [in] filename: file path with joint planner settings.
     * @param [in] fieldName: field where settings are defined
     * @param [in] verbose: verbose flag
     * @return multi_end_effector_kinematics::InverseSolverSettings struct
     */
    multi_end_effector_kinematics::InverseSolverSettings loadInverseSolverSettings(
      const std::string& filename, 
      const std::string& fieldName = "inverse_solver_settings", 
      bool verbose = true);

     /**
     * Creates InverseSolver model name
     * @param [in] filename: file path with joint planner settings.
     * @param [in] fieldName: field where settings are defined
     * @param [in] verbose: verbose flag
     * @return Inverse solver name string
     */
    std::string loadInverseSolverName(const std::string& filename, 
      const std::string& fieldName = "inverse_solver_name", 
      bool verbose = true);
  };
} // namespace legged_locomotion_mpc

#endif