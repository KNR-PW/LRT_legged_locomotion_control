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

#include <terrain_model/core/TerrainModel.hpp>

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

namespace legged_locomotion_mpc_ros2
{
  using namespace legged_locomotion_mpc;

  class LeggedMpcController: public rclcpp_lifecycle::LifecycleNode
  {
    public:

      LeggedMpcController(bool intraProcessComms = false);

      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State& state);

      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State& state);

      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State& state);

      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_cleanup(const rclcpp_lifecycle::State& state);

      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_shutdown(const rclcpp_lifecycle::State& state);

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

      void sendJointTrajectory();

      void setupMpc();

      void advanceMpc();

      ocs2::SystemObervation getCurrentObservation();

      void sendMrtCommands();

      /**
       * Parameter variables
       */ 

      // Parameter for directory of all config files (model, task and loopshaping definition)
      std::string configDirectoryPath_;
      

      /**
       * Non - parameter variables
       */ 

      std::unordered_map<std::string, size_t> jointNameIndexMap_;

      std::unordered_map<std::string, size_t> contactFrameNameIndexMap_;

      std::string robotName_;
      rclcpp::Time initialTime_;
      ModelSettings modelSettings_;
      floating_base_model::FloatingBaseModelInfo modelInfo_;
      ocs2::scalar_t mrtDuation_;
      size_t endEffectorNum_;
      size_t stateOffset_;
      size_t inputOffset_;
      std::vector<std::string> jointNames_;
      std::unique_ptr<terrain_model::TerrainModel> terrainModelPtr_;

      ocs2::BufferedValue<vector6_t> basePoseEstimation_;
      ocs2::BufferedValue<vector6_t> baseTwistEstimation_;
      ocs2::BufferedValue<vector_t> jointPositionEstimation_;
      ocs2::BufferedValue<vector_t> jointVelocityEstimation_;
      ocs2::SystemObservation currentLoopshapingObservation_;


      std::unique_ptr<LeggedLoopshapingInterface> loopshapingInterfacePtr_;
      LeggedInterface* leggedInterfacePtr_;
      LeggedReferenceManager* referenceManagerPtr_;
      LoopshapingDefinition* loopshapingDefinitionPtr_;
      
      std::unique_ptr<ocs2::MPC_BASE> mpcPtr_;
      std::unique_ptr<ocs2::MPC_MRT_Interface> mpcMrtPtr_;

      std::thread mpcThread_;
      std::thread mrtThread_;

      std::atomic_bool controllerRunning_, mpcRunning_;

      rclcpp::TimerBase::SharedPtr jointTrajectoryTimer_;

      rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr commandTwistSubscriber_;
      rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointStateSubscriber_;
      rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr basePoseSubscriber_;
      rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr baseTwistSubscriber_;
      rclcpp::Subscription<contact_msgs::msg::Contacts> contactsSubscriber_;
      rclcpp::Subscription<legged_locomotion_msgs::msg::GaitParameters> gaitParametersSubscriber_;
      rclcpp::Subscription<legged_locomotion_msgs::msg::SwingParameters> swingParametersSubscriber_;
      rclcpp::Subscription<geometry_msgs::msg::WrenchStamped> baseWrenchSubscriber_;

      std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<
        trajectory_msgs::msg::JointTrajectory>> jointTrajectoryPublisher_;

      // Last time robot state messages was recived
      rclcpp::Time lastJointStateTime_;
      rclcpp::Time lastBasePoseTime_;
      rclcpp::Time lastBaseTwistTime_;
      rclcpp::Time lastContactFlagsTime_;
      
      // Maximum duration between robot state messages
      rclcpp::Duration maxDurationBetweenMessages_;
  };  
} // namespace legged_locomotion_mpc_ros2


#endif