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

#ifndef __LEGGED_STATE_ESTIMATOR_LEGGED_STATE_ESTIMATOR_ROS2__
#define __LEGGED_STATE_ESTIMATOR_LEGGED_STATE_ESTIMATOR_ROS2__

#include <ocs2_core/Types.h>

#include <legged_state_estimator/legged_state_estimator.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

namespace legged_state_estimator_ros2
{
  using namespace legged_state_estimator;

  class LeggedStateEstimator: public rclcpp_lifecycle::LifecycleNode
  {
    public:

      LeggedStateEstimator();

      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State& state) override;

      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State& state) override;

      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State& state) override;

    private:

      void updateCurrentSensorData();

      void sendStateEstimations();

      // Helper data
      size_t endEffectorNum_;
      std::vector<std::string> jointNames_;
      std::unordered_map<std::string, size_t> jointNameIndexMap_;
      std::unordered_map<std::string, size_t> contactFrameNameIndexMap_;
      
      // Data from sensors / observations (IMU, joint states, contact sensors)
      ocs2::vector_t jointPositions_;
      ocs2::vector_t jointVelocities_;
      ocs2::vector_t jointTorques_;
      quaterion_t quaterion_;
      vector3_t angularVelocity_;
      vector3_t linearAcceleration_;
      std::vector<std::pair<int, bool>> contactFlags_;
      
      // Legged state estimator
      bool estimatorRunning_;
      std::unique_ptr<legged_state_estimator::LeggedStateEstimator> leggedStateEstimator_;

      // Timer for state estimator loop
      ocs2::scalar_t estimatorDuration_;
      rclcpp::TimerBase::SharedPtr estimatorLoopTimer_;

      // Sensor / observation subscribers
      rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointStateSubscriber_;
      rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSubscriber_;
      rclcpp::Subscription<contact_msgs::msg::Contacts>::SharedPtr contactsSubscriber_;

      // Base and joint state estimation publishers
      std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<
        geometry_msgs::msg::TransformStamped> baseTransformEstimatePublisher_;
      std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<
        geometry_msgs::msg::TwistStamped> baseTwistEstimatePublisher_;
      std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<
        sensor_msgs::msg::JointState> jointStatesEstimatePublisher_;

      // Last time robot sensor messages were recived
      rclcpp::Time lastJointStateTime_;
      rclcpp::Time lastImuTime_;
      rclcpp::Time lastContactFlagsTime_;
      
      // Maximum duration between robot sensor messages
      rclcpp::Duration maxDurationBetweenMessages_ = rclcpp::Duration(1, 0);
  };  
} // namespace legged_state_estimator_ros2


#endif