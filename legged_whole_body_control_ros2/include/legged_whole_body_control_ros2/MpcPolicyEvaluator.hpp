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

#ifndef __MPC_POLICY_EVALUATOR_LEGGED_WHOLE_BODY_CONTROL_ROS2__
#define __MPC_POLICY_EVALUATOR_LEGGED_WHOLE_BODY_CONTROL_ROS2__

#include <memory>
#include <string>
#include <vector>

#include <ocs2_core/Types.h>
#include <ocs2_mpc/MRT_BASE.h>

#include <legged_whole_body_control/Types.hpp>

#include <legged_locomotion_msgs/msg/mpc_policy.hpp>

namespace legged_whole_body_control_ros2
{
  /**
   * MRT of the whole body controller: MPC policy messages are validated and moved into
   * ocs2::MRT_BASE, which evaluates the policy at WBC time. Terrain normals of the policy
   * are kept next to it.
   */
  class MpcPolicyEvaluator
  {
    public:

      /**
       * @param [in] stateDim: MPC state dimension
       * @param [in] inputDim: MPC input dimension
       * @param [in] endEffectorNum: number of end effectors (3 DoF and 6 DoF)
       */
      MpcPolicyEvaluator(size_t stateDim, size_t inputDim, size_t endEffectorNum);

      /**
       * Reads policy message, only feedforward controller is supported. All policy times are shifted
       * by timeOffset. Invalid message is ignored and the previous policy is kept.
       * @param [in] message: MPC policy message
       * @param [in] timeOffset: time added to policy times (MPC time -> caller time) [s]
       * @param [out] error: reason why message is invalid
       * @return true if policy was replaced
       */
      bool readPolicy(const legged_locomotion_msgs::msg::MpcPolicy& message,
        ocs2::scalar_t timeOffset, std::string& error);

      /** Removes the policy */
      void reset();

      bool hasPolicy() const { return hasPolicy_; }

      /** Start time of the policy (MPC initial time), policy has to be available */
      ocs2::scalar_t getStartTime() const;

      /** Final time of the policy, policy has to be available */
      ocs2::scalar_t getFinalTime() const;

      /**
       * Desired state and input at time (ocs2::MRT_BASE::evaluatePolicy), zero order extrapolation
       * outside of the policy. Policy has to be available.
       * @param [in] time: query time
       * @param [in] measuredState: measured MPC state (not used by feedforward controller)
       * @param [out] state: desired state
       * @param [out] input: desired input
       */
      void evaluate(ocs2::scalar_t time, const ocs2::vector_t& measuredState,
        ocs2::vector_t& state, ocs2::vector_t& input);

      /** Planned contact flags at time (from mode schedule), policy has to be available */
      legged_whole_body_control::contact_flags_t getContactFlags(ocs2::scalar_t time) const;

      /**
       * Terrain normals of the last policy time point not later than time (first point if time
       * is before the policy). Policy has to be available.
       */
      const std::vector<legged_whole_body_control::vector3_t>& getTerrainNormals(
        ocs2::scalar_t time) const;

    private:

      /**
       * MRT that gets policy directly from the evaluator. It does not communicate with MPC:
       * MPC is not reset by WBC and gets robot state from the state estimator.
       */
      class PolicyMrt final: public ocs2::MRT_BASE
      {
        public:

          void resetMpcNode(const ocs2::TargetTrajectories&) override {}

          void setCurrentObservation(const ocs2::SystemObservation&) override {}

          // moveToBuffer is protected in MRT_BASE
          void setNewPolicy(std::unique_ptr<ocs2::CommandData> command,
            std::unique_ptr<ocs2::PrimalSolution> policy,
            std::unique_ptr<ocs2::PerformanceIndex> performanceIndices)
          {
            moveToBuffer(std::move(command), std::move(policy), std::move(performanceIndices));
          }
      };

      // Index of the last time point not later than time, 0 if time is before the policy
      size_t findTimePointIndex(ocs2::scalar_t time) const;

      const size_t stateDim_;
      const size_t inputDim_;
      const size_t endEffectorNum_;

      // Active policy of MRT is available
      bool hasPolicy_ = false;

      PolicyMrt mrt_;

      // Normals of end effectors at every time point of the active MRT policy
      std::vector<std::vector<legged_whole_body_control::vector3_t>> terrainNormals_;

      // Normals of the policy moved into MRT buffer, swapped when MRT policy is updated
      std::vector<std::vector<legged_whole_body_control::vector3_t>> newTerrainNormals_;
  };
} // namespace legged_whole_body_control_ros2

#endif
