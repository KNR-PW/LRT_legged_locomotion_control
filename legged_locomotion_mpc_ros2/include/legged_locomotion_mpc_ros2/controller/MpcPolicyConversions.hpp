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

#ifndef __MPC_POLICY_CONVERSIONS_LEGGED_LOCOMOTION_MPC_ROS2__
#define __MPC_POLICY_CONVERSIONS_LEGGED_LOCOMOTION_MPC_ROS2__

#include <ocs2_mpc/CommandData.h>
#include <ocs2_oc/oc_data/PerformanceIndex.h>
#include <ocs2_oc/oc_data/PrimalSolution.h>

#include <ocs2_msgs/msg/mpc_flattened_controller.hpp>

namespace legged_locomotion_mpc_ros2
{
  /**
   * Writes MPC policy into flattened controller message as feedforward controller
   * (optimal input at every time point). Feedback gains are not sent, so the message
   * is small and the same for every MPC solver. Target trajectories are not sent.
   * Message vectors are reused, so a message kept between calls does not allocate them again.
   * @param [in] policy: MPC policy
   * @param [in] command: command data of the policy
   * @param [in] performanceIndices: performance indices of the policy
   * @param [out] message: flattened controller message
   */
  void createMpcPolicyMessage(const ocs2::PrimalSolution& policy,
    const ocs2::CommandData& command, const ocs2::PerformanceIndex& performanceIndices,
    ocs2_msgs::msg::MpcFlattenedController& message);
} // namespace legged_locomotion_mpc_ros2

#endif
