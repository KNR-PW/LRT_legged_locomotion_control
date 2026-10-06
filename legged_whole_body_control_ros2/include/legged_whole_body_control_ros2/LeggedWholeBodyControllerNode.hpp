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

#ifndef __LEGGED_WHOLE_BODY_CONTROLLER_LEGGED_WHOLE_BODY_CONTROL_ROS2__
#define __LEGGED_WHOLE_BODY_CONTROLLER_LEGGED_WHOLE_BODY_CONTROL_ROS2__

#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_mpc/SystemObservation.h>

#include <ocs2_core/misc/Benchmark.h>

#include <floating_base_model/FloatingBaseModelInfo.hpp>

#include <legged_whole_body_control/Types.hpp>
#include <legged_whole_body_control/WbcBase.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include <sensor_msgs/msg/joint_state.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <ocs2_msgs/msg/mpc_observation.hpp>

#include <contact_msgs/msg/contacts.hpp>

namespace legged_whole_body_control_ros2
{
  class LeggedWholeBodyControllerNode: public rclcpp_lifecycle::LifecycleNode
  {
    public:

      LeggedWholeBodyControllerNode();

      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State& state) override;

      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State& state) override;

      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State& state) override;

    private:

      void updateExternalWrench(
        const geometry_msgs::msg::WrenchStamped::ConstSharedPtr externalWrench);

      void sendJointTrajectory();

      void setupWbc();
      
      void updateMeasuredObservation();
      void updateDesiredObservation();

      // Helper data
      floating_base_model::FloatingBaseModelInfo modelInfo_;

      std::vector<std::string> jointNames_;
      std::unordered_map<std::string, size_t> jointNameIndexMap_;
      

      std::unique_ptr<ocs2::MPC_BASE> mpcPtr_;
      std::unique_ptr<ocs2::MPC_MRT_Interface> mpcMrtPtr_;

      // Rollout for getting future optimal trajectory
      std::unique_ptr<ocs2::RolloutBase> rolloutPtr_;

      std::atomic_bool controllerRunning_;

      // Timer for MRT
      ocs2::scalar_t mrtDurationSeconds_;
      rclcpp::TimerBase::SharedPtr jointTrajectoryTimer_;

      // Observation subscribers
      rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointStateSubscriber_;
      rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr baseTransformSubscriber_;
      rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr baseTwistSubscriber_;
      rclcpp::Subscription<ocs2_msgs::msg::MpcObservation>::SharedPtr desiredObservationSubscriber_;
      
      // Command subscribers
      rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr baseWrenchSubscriber_;
      
      // Joint trajectory publisher from MRT
      std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<
        trajectory_msgs::msg::JointTrajectory>> jointTrajectoryPublisher_;

      // Last time robot state messages was recived
      rclcpp::Time lastJointStateTime_;
      rclcpp::Time lastBaseTransformTime_;
      rclcpp::Time lastBaseTwistTime_;
      rclcpp::Time lastContactFlagsTime_;

      // WBC Base
      std::unique_ptr<legged_whole_body_control::WbcBase> wbcPtr_;

      // System observations
      ocs2::SystemObservation currentMeasuredObservation_;
      ocs2::SystemObservation currentDesiredObservation_;
      
      // Maximum duration between robot state messages
      rclcpp::Duration maxDurationBetweenMessages_ = rclcpp::Duration(1, 0);
      
      // Timers
      ocs2::benchmark::RepeatedTimer wbcTimer_;

      // Decomposition pipeline for segmented terrain model
      std::unique_ptr<convex_plane_decomposition::PlaneDecompositionPipeline> decompositionPipelinePtr_;
  };  
} // namespace legged_whole_body_control_ros2


#endif