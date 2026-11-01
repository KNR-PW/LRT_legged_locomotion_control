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
 * Based on: Farbod Farshidian (https://github.com/leggedrobotics/ocs2)
 */

#ifndef __LEGGED_VISUALIZER_LEGGED_LOCOMOTION_MPC_ROS2__
#define __LEGGED_VISUALIZER_LEGGED_LOCOMOTION_MPC_ROS2__

#include <ocs2_core/Types.h>
#include <ocs2_core/reference/ModeSchedule.h>
#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_ros_interfaces/visualization/VisualizationColors.h>

#include <floating_base_model/FloatingBaseModelInfo.hpp>

#include <terrain_model/core/TerrainModel.hpp>

#include <legged_locomotion_mpc/common/Types.hpp>
#include <legged_locomotion_mpc/common/ModelSettings.hpp>
#include <legged_locomotion_mpc/kinematics/PinocchioForwardEndEffectorKinematicsCppAd.hpp>
#include <legged_locomotion_mpc/torque_approx/PinocchioTorqueApproximationCppAd.hpp>

#include <rclcpp/node.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <grid_map_ros/GridMapRosConverter.hpp>

namespace legged_locomotion_mpc_ros2
{
  class LeggedVisualizer: public rclcpp::Node
  {
    public:

      struct Settings
      {
        /* Time difference between published data */
        ocs2::scalar_t deltaTime = 0.05;

        /* Size of the spheres at the end effector */
        ocs2::scalar_t endEffectorMarkerDiameter = 0.03;

        /* Alpha value when a foot is lifted */
        ocs2::scalar_t endEffectorAlphaWhenLifted = 0.3; 

        /* Vector scale in N/m */
        ocs2::scalar_t forceScale = 500.0;

         /* Vector scale in N/m */
        ocs2::scalar_t torqueScale = 500.0;

        /* Vector scale in m/s */
        ocs2::scalar_t velocityScale = 5.0;

        /* Size of the sphere at the center of pressure */
        ocs2::scalar_t centerOfPressureMarkerDiameter = 0.03; 

        /* Line thickness for the support polygon */
        ocs2::scalar_t supportPolygonLineWidth = 0.005;

        /* Line thickness for trajectories */
        ocs2::scalar_t trajectoryLineWidth = 0.01;

        /* End effector collor map, used for all end effectors iteratively */
        std::vector<ocs2::Color> endEffectorColorMap = 
        {
          ocs2::Color::blue, ocs2::Color::orange, 
          ocs2::Color::yellow, ocs2::Color::purple
        };
      };

      LeggedVisualizer(Settings visualizationSettings,
        legged_locomotion_mpc::ModelSettings modelSettings,
        floating_base_model::FloatingBaseModelInfo modelInfo,
        const pinocchio::Model& robotModel, 
        const legged_locomotion_mpc::PinocchioForwardEndEffectorKinematicsCppAd& 
          forwardKinematics, 
        const legged_locomotion_mpc::PinocchioTorqueApproximationCppAd& 
          torqueApproximator);

      void publishObservation(const rclcpp::Time& timeStamp, 
        const ocs2::SystemObservation& currentObservation);

      void publishTargetTrajectory(const rclcpp::Time& timeStamp,
        const ocs2::TargetTrajectories& targetTrajectory);

      void publishOptimizedTrajectory(const rclcpp::Time& timeStamp, 
        const ocs2::scalar_array_t& optimizedTimeTrajectory,
        const ocs2::vector_array_t& optimizedStateTrajectory);

      void publishTerrainModel(const terrain_model::TerrainModel& terrainModel);

    private:

      void publishJointStates(const rclcpp::Time& timeStamp, 
        const ocs2::vector_t& jointPositons, const ocs2::vector_t& jointVelocities, 
        const ocs2::vector_t& jointTorques);
      
      void publishBaseTransform(const rclcpp::Time& timeStamp, std::string frameName, 
        const legged_locomotion_mpc::vector3_t& basePositon, 
        const legged_locomotion_mpc::vector3_t& baseEulerAngles);

      void publishBaseTransform(const rclcpp::Time& timeStamp,  
        const legged_locomotion_mpc::vector3_t& basePositon, 
        const legged_locomotion_mpc::vector3_t& baseEulerAngles);

      void publishBaseTwist(const rclcpp::Time& timeStamp, 
        const legged_locomotion_mpc::vector3_t& baseLinearVelocity, 
        const legged_locomotion_mpc::vector3_t& baseAngularVelocity);

      void publishEndEffectorMarkers(const rclcpp::Time& timeStamp, 
        const legged_locomotion_mpc::contact_flags_t& contactFlags,
        const std::vector<legged_locomotion_mpc::vector3_t>& endEffectorPositions,
        const std::vector<legged_locomotion_mpc::vector3_t>& endEffectorForces, 
        const std::vector<legged_locomotion_mpc::vector3_t>& endEffectorTorques);
      
      const Settings visualizationSettings_;
      const legged_locomotion_mpc::ModelSettings modelSettings_;
      const floating_base_model::FloatingBaseModelInfo modelInfo_;
      const std::string robotName_;
      std::vector<std::string> jointNames_;

      std::shared_ptr<tf2_ros::TransformBroadcaster> baseTransformBroadcaster_;
      
      rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr baseTwistPublisher_;

      rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointStatePublisher_;

      rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr 
        desiredBasePositionPublisher_;

      std::vector<rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr>
        desiredEndEffectorDataPublishers_;

      rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr 
        optimizedBasePositionPublisher_;

      std::vector<rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr>
        optimizedEndEffectorDataPublishers_;
        
      rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
        currentStatePublisher_; 

      rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr gridMapPublisher_;

      const legged_locomotion_mpc::PinocchioForwardEndEffectorKinematicsCppAd 
        forwardKinematics_;
      const legged_locomotion_mpc::PinocchioTorqueApproximationCppAd 
        torqueApproximator_;
  };
} // namespace legged_locomotion_mpc_ros2

#endif