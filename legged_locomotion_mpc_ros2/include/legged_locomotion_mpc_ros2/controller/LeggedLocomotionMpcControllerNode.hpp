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

#ifndef __LEGGED_MPC_CONTROLLER_LEGGED_LOCOMOTION_MPC_ROS2__
#define __LEGGED_MPC_CONTROLLER_LEGGED_LOCOMOTION_MPC_ROS2__

#include <atomic>
#include <mutex>
#include <thread>

#include <ocs2_core/Types.h>
#include <ocs2_core/reference/ModeSchedule.h>
#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_mpc/SystemObservation.h>

#include <ocs2_mpc/MPC_MRT_Interface.h>

#include <ocs2_core/misc/Benchmark.h>

#include <terrain_model/core/TerrainModel.hpp>

#include <pinocchio/fwd.hpp>
#include <convex_plane_decomposition/PlaneDecompositionPipeline.h>

#include <legged_locomotion_mpc/robot_interface/LeggedLoopshapingInterface.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include <sensor_msgs/msg/joint_state.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <contact_msgs/msg/contacts.hpp>

#include <legged_locomotion_msgs/msg/gait_parameters.hpp>
#include <legged_locomotion_msgs/msg/swing_parameters.hpp>

#include <grid_map_ros/GridMapRosConverter.hpp>

namespace legged_locomotion_mpc_ros2
{
  using namespace legged_locomotion_mpc;

  class LeggedLocomotionMpcControllerNode: public rclcpp_lifecycle::LifecycleNode
  {
    public:

      LeggedLocomotionMpcControllerNode();

      ~LeggedLocomotionMpcControllerNode() override;

      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State& state) override;

      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State& state) override;

      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State& state) override;

      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_shutdown(const rclcpp_lifecycle::State& state) override;

      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_error(const rclcpp_lifecycle::State& state) override;

    private:

      void updateCommand(
        const geometry_msgs::msg::TwistStamped::ConstSharedPtr baseTwist);

      void updateGaitParameters(
        const legged_locomotion_msgs::msg::GaitParameters::ConstSharedPtr gaitParameters);

      void updateSwingParameters(
        const legged_locomotion_msgs::msg::SwingParameters::ConstSharedPtr swingParameters);

      void updateExternalWrench(
        const geometry_msgs::msg::WrenchStamped::ConstSharedPtr externalWrench);

      void updateSegmentedTerrain(
        grid_map_msgs::msg::GridMap::UniquePtr gridMap);

      void sendJointTrajectory();

      void setTrajectoryPoint(size_t pointIndex, const ocs2::vector_t& state,
        const ocs2::vector_t& input);

      bool waitForInitialState();

      void setupMpc();

      void runMpc();
      
      void stopMpcThread();

      void updateCurrentObservation();

      void updateOdometry();

      void updateJointStates();

      void updateContacts();

      bool isStateFresh(const rclcpp::Time& currentTime) const;

      bool isOdometryFresh(const rclcpp::Time& currentTime) const;

      bool isJointStateFresh(const rclcpp::Time& currentTime) const;

      // Helper data
      ModelSettings modelSettings_;
      floating_base_model::FloatingBaseModelInfo modelInfo_;
      ocs2::scalar_t minimumSteppingFrequency_ = 0.0;

      size_t endEffectorNum_ = 0;
      std::vector<std::string> jointNames_;
      std::unordered_map<std::string, size_t> jointNameIndexMap_;
      std::unordered_map<std::string, size_t> contactFrameNameIndexMap_;
      
      // Data from observation
      contact_flags_t contactFlags_;
      bool newContactFlags_ = false;

      // Terrain for next MPC setup, guarded by terrainMutex_
      std::unique_ptr<terrain_model::TerrainModel> terrainModelPtr_;
      
      // Interface and other helper objects
      std::unique_ptr<LeggedInterface> leggedInterfacePtr_;
      LeggedReferenceManager* referenceManagerPtr_ = nullptr;
      
      // MPC and MRT
      std::unique_ptr<ocs2::MPC_BASE> mpcPtr_;
      std::unique_ptr<ocs2::MPC_MRT_Interface> mpcMrtPtr_;

      // MPC thread
      std::thread mpcThread_;
      std::atomic_bool controllerRunning_{false};

      // Set by MPC thread on error, deactivation is done by MRT timer
      std::atomic_bool mpcFailed_{false};

      // MPC loop duration (1 / mpcFreuency)
      rclcpp::Duration mpcDuration_ = rclcpp::Duration(0, 0);

      // Timer for MRT
      ocs2::scalar_t mrtDurationSeconds_ = 0.0;
      rclcpp::TimerBase::SharedPtr jointTrajectoryTimer_;

      // Terrain processing is slow, separate group so it does not block MRT timer
      rclcpp::CallbackGroup::SharedPtr terrainCallbackGroup_;
      std::mutex terrainMutex_;

      // Observation subscribers
      rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointStateSubscriber_;
      rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometrySubscriber_;
      rclcpp::Subscription<contact_msgs::msg::Contacts>::SharedPtr contactsSubscriber_;
      rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr terrainSubscriber_;
      
      // Command subscribers
      rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr commandTwistSubscriber_;
      rclcpp::Subscription<legged_locomotion_msgs::msg::GaitParameters>::SharedPtr gaitParametersSubscriber_;
      rclcpp::Subscription<legged_locomotion_msgs::msg::SwingParameters>::SharedPtr swingParametersSubscriber_;
      rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr baseWrenchSubscriber_;
      
      // Joint trajectory publisher from MRT
      std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<
        trajectory_msgs::msg::JointTrajectory>> jointTrajectoryPublisher_;

      // Preallocated in on_configure, only point values change in MRT loop
      trajectory_msgs::msg::JointTrajectory jointTrajectory_;

      // Stamps of last robot state messages
      rclcpp::Time lastJointStateTime_;
      rclcpp::Time lastOdometryTime_;

      // Robot state messages received since activation started
      bool jointStateReceived_ = false;
      bool odometryReceived_ = false;

      // Reused by every MRT tick, take() overwrites them without new allocations
      sensor_msgs::msg::JointState jointStateMessage_;
      nav_msgs::msg::Odometry odometryMessage_;
      contact_msgs::msg::Contacts contactsMessage_;
      ocs2::vector_t jointPositionsBuffer_;
      std::vector<bool> jointUpdatedBuffer_;
      std::vector<bool> contactUpdatedBuffer_;

      ocs2::SystemObservation currentObservation_;

      // MPC time is relative to this ROS time, absolute ROS time in double is too coarse
      rclcpp::Time mpcStartTime_;

      // Maximum age of odometry and joint states messages
      rclcpp::Duration odometryTimeout_ = rclcpp::Duration(0, 0);
      rclcpp::Duration jointStateTimeout_ = rclcpp::Duration(0, 0);

      // Maximum age of MPC policy used by MRT [s]
      ocs2::scalar_t policyTimeout_ = 0.0;

      // Maximum duration between base velocity commands
      rclcpp::Duration velocityTimeout_ = rclcpp::Duration(0, 0);
      rclcpp::Time lastCommandTime_;
      bool commandActive_ = false;

      // Maximum duration between external wrench messages
      rclcpp::Duration wrenchTimeout_ = rclcpp::Duration(0, 0);
      rclcpp::Time lastWrenchTime_;
      bool wrenchActive_ = false;
      
      // Benchmark timers
      ocs2::benchmark::RepeatedTimer mpcBenchmarkTimer_;
      ocs2::benchmark::RepeatedTimer mrtBenchmarkTimer_;

      // Decomposition pipeline for segmented terrain model, pointer guarded by terrainMutex_.
      // Shared, so terrain callback can use it without holding terrainMutex_
      std::shared_ptr<convex_plane_decomposition::PlaneDecompositionPipeline> decompositionPipelinePtr_;
  };  
} // namespace legged_locomotion_mpc_ros2


#endif