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

#include <geometry_msgs/msg/pose_stamped.hpp>
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

  class LeggedMpcController: public rclcpp_lifecycle::LifecycleNode
  {
    public:

      LeggedMpcController(bool intraProcessComms = false);

      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State& state) override;

      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State& state) override;

      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State& state) override;

    private:

      void updateCommand(
        const geometry_msgs::msg::TwistStamped::ConstSharedPtr baseTwist);

      void updateJointStates(
        const sensor_msgs::msg::JointState::ConstSharedPtr jointStates);

      void updateBasePose(
        const geometry_msgs::msg::PoseStamped::ConstSharedPtr basePose);

      void updateBaseTwist(
        const geometry_msgs::msg::TwistStamped::ConstSharedPtr baseTwist);

      void updateContactFlags(
        const contact_msgs::msg::Contacts::ConstSharedPtr contactFlags);

      void updateGaitParameters(
        const legged_locomotion_msgs::msg::GaitParameters::ConstSharedPtr gaitParameters);

      void updateSwingParameters(
        const legged_locomotion_msgs::msg::SwingParameters::ConstSharedPtr swingParameters);

      void updateExternalWrench(
        const geometry_msgs::msg::WrenchStamped::ConstSharedPtr externalWrench);

      void updateSegmentedTerrain(
        grid_map_msgs::msg::GridMap::UniquePtr gridMap);

      void sendJointTrajectory();

      void setupMpc();

      void runMpc();

      ocs2::SystemObservation getCurrentObservation();

      // Helper data
      ModelSettings modelSettings_;
      floating_base_model::FloatingBaseModelInfo modelInfo_;

      size_t endEffectorNum_;
      std::vector<std::string> jointNames_;
      std::unordered_map<std::string, size_t> jointNameIndexMap_;
      std::unordered_map<std::string, size_t> contactFrameNameIndexMap_;
      
      // Data from observation
      ocs2::BufferedValue<vector6_t> basePoseEstimation_;
      ocs2::BufferedValue<vector6_t> baseTwistEstimation_;
      ocs2::BufferedValue<ocs2::vector_t> jointPositionEstimation_;
      ocs2::BufferedValue<ocs2::vector_t> jointVelocityEstimation_;
      std::unique_ptr<terrain_model::TerrainModel> terrainModelPtr_;
      
      // Loopshaping interface and other helper objects
      std::unique_ptr<LeggedLoopshapingInterface> loopshapingInterfacePtr_;
      LeggedInterface* leggedInterfacePtr_;
      LeggedReferenceManager* referenceManagerPtr_;
      ocs2::LoopshapingDefinition* loopshapingDefinitionPtr_;
      
      // MPC and MRT
      std::unique_ptr<ocs2::MPC_BASE> mpcPtr_;
      std::unique_ptr<ocs2::MPC_MRT_Interface> mpcMrtPtr_;

      // Rollout for getting future optimal trajectory
      std::unique_ptr<ocs2::RolloutBase> rolloutPtr_;

      // MPC thread
      std::thread mpcThread_;
      std::atomic_bool controllerRunning_;

      // Timer for MRT
      ocs2::scalar_t mrtDuration_;
      rclcpp::TimerBase::SharedPtr jointTrajectoryTimer_;

      // Observation subscribers
      rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointStateSubscriber_;
      rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr basePoseSubscriber_;
      rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr baseTwistSubscriber_;
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

      // Last time robot state messages was recived
      rclcpp::Time lastJointStateTime_;
      rclcpp::Time lastBasePoseTime_;
      rclcpp::Time lastBaseTwistTime_;
      rclcpp::Time lastContactFlagsTime_;
      
      // Maximum duration between robot state messages
      rclcpp::Duration maxDurationBetweenMessages_ = rclcpp::Duration(1, 0);
      
      // Timers
      ocs2::benchmark::RepeatedTimer mpcTimer_;
      ocs2::benchmark::RepeatedTimer mrtTimer_;

      // Decomposition pipeline for segmented terrain model
      std::unique_ptr<convex_plane_decomposition::PlaneDecompositionPipeline> decompositionPipelinePtr_;
  };  
} // namespace legged_locomotion_mpc_ros2


#endif