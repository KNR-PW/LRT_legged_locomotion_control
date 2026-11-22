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

#include <atomic>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

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

#include <geometry_msgs/msg/wrench_stamped.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <legged_locomotion_msgs/msg/mpc_policy.hpp>

#include <legged_whole_body_control_ros2/MpcPolicyEvaluator.hpp>

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
      
      bool waitForInitialState();

      bool isStateFresh(const rclcpp::Time& currentTime) const;

      bool isOdometryFresh(const rclcpp::Time& currentTime) const;

      bool isJointStateFresh(const rclcpp::Time& currentTime) const;

      void updateMeasuredObservation();
      void updateOdometry();
      void updateJointStates();

      // Reads new MPC policy message (if there is one) into policy evaluator
      void updatePolicy();

      // Policy is available, not older than policy_timeout and time is not after its end
      bool isPolicyValid(ocs2::scalar_t time) const;

      // Standing reference from measured state: zero velocities, all end effectors in contact,
      // weight compensating contact forces. Used when there is no valid MPC policy
      void setStandingReference();

      // Helper data
      floating_base_model::FloatingBaseModelInfo modelInfo_;
      legged_whole_body_control::WbcBase::Settings wbcSettings_;

      std::vector<std::string> jointNames_;
      std::unordered_map<std::string, size_t> jointNameIndexMap_;
      std::atomic_bool controllerRunning_{false};
      size_t endEffectorNum_ = 0;

      // Robot model from URDF, made in on_configure and used for WBC setup on every activation
      std::unique_ptr<ocs2::PinocchioInterface> pinocchioInterfacePtr_;

      // Observation subscribers
      rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointStateSubscriber_;
      rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometrySubscriber_;

      // Command subscribers
      rclcpp::Subscription<legged_locomotion_msgs::msg::MpcPolicy>::SharedPtr policySubscriber_;
      rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr baseWrenchSubscriber_;

      // Reused by every WBC tick, take() overwrites it
      legged_locomotion_msgs::msg::MpcPolicy policyMessage_;
      
      // Joint trajectory publisher from MRT
      std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<
        trajectory_msgs::msg::JointTrajectory>> jointTrajectoryPublisher_;

      // Preallocated in on_configure, only point values change in WBC loop
      trajectory_msgs::msg::JointTrajectory jointTrajectory_;

      // WBC loop timer
      ocs2::scalar_t wbcDurationSeconds_ = 0.0;
      rclcpp::TimerBase::SharedPtr jointTrajectoryTimer_;

      // Last time robot state messages was recived
      rclcpp::Time lastJointStateTime_;
      rclcpp::Time lastOdometryTime_;

      // WBC time is relative to this ROS time, absolute ROS time in double is too coarse
      rclcpp::Time wbcStartTime_;

      // Robot state messages received since activation started
      bool jointStateReceived_ = false;
      bool odometryReceived_ = false;

      // WBC Base
      std::unique_ptr<legged_whole_body_control::WbcBase> wbcPtr_;

      // MPC policy (MRT of the whole body controller), created in on_configure
      std::unique_ptr<MpcPolicyEvaluator> policyEvaluatorPtr_;

      // Maximum age of MPC policy (from its start time), WBC is standing after that
      ocs2::scalar_t policyTimeout_ = 0.0;

      // WBC follows MPC policy, false when standing
      bool followingPolicy_ = false;

      // Measured observation, time is relative to wbcStartTime_
      ocs2::SystemObservation currentMeasuredObservation_;

      // Standing reference, used without valid MPC policy
      ocs2::vector_t standingState_;
      ocs2::vector_t standingInput_;

      // Desired state and input one WBC period before current time, at current time
      // and one WBC period after current time (joint command)
      ocs2::vector_t previousDesiredState_;
      ocs2::vector_t previousDesiredInput_;
      ocs2::vector_t desiredState_;
      ocs2::vector_t desiredInput_;
      ocs2::vector_t nextDesiredState_;
      ocs2::vector_t nextDesiredInput_;

      // Terrain normals and contact flags passed to WBC
      std::vector<legged_whole_body_control::vector3_t> terrainNormals_;
      legged_whole_body_control::contact_flags_t contactFlags_;
      
      // Maximum age of odometry and joint states messages, WBC is deactivated after that
      rclcpp::Duration odometryTimeout_ = rclcpp::Duration(0, 0);
      rclcpp::Duration jointStateTimeout_ = rclcpp::Duration(0, 0);
      
      // Maximum duration between external wrench messages, wrench is zeroed after that
      rclcpp::Duration wrenchTimeout_ = rclcpp::Duration(0, 0);
      rclcpp::Time lastWrenchTime_;
      bool wrenchActive_ = false;

      // WBC is deactivated after this many not solved problems in a row
      size_t maxConsecutiveFailures_ = 0;
      size_t consecutiveFailures_ = 0;

      // Benchmark timers
      ocs2::benchmark::RepeatedTimer wbcBenchmarkTimer_;
  };  
} // namespace legged_whole_body_control_ros2


#endif