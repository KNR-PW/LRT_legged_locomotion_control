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

#include <deque>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <legged_state_estimator/legged_state_estimator.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <contact_msgs/msg/contacts.hpp>

namespace legged_state_estimator_ros2
{
  using namespace legged_state_estimator;

  class LeggedStateEstimatorNode: public rclcpp_lifecycle::LifecycleNode
  {
    public:

      LeggedStateEstimatorNode();

      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State& state) override;

      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State& state) override;

      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State& state) override;

    private:

      void processImu(const sensor_msgs::msg::Imu::ConstSharedPtr imu);

      void processJointStates(const sensor_msgs::msg::JointState::ConstSharedPtr jointStates);

      ocs2::scalar_t toEstimatorTime(const rclcpp::Time& time) const;

      rclcpp::Time fromEstimatorTime(const ocs2::scalar_t time) const;

      void checkSensorTimeouts();

      void publishEstimates(const rclcpp::Time& stamp);

      bool waitForInitialData();

      std::string getMissingInitialData(const rclcpp::Time& currentTime) const;

      bool isImuFresh(const rclcpp::Time& currentTime) const;

      bool isJointStateFresh(const rclcpp::Time& currentTime) const;

      bool isContactsFresh(const rclcpp::Time& currentTime) const;

      void updateInitialOdometry();

      /** Adds the last read IMU measurement to the bias calibration window */
      void addImuCalibrationSample();

      /** IMU measurements in the calibration window span at least imu_bias_calibration_duration */
      bool isImuCalibrationReady() const;

      /**
       * IMU biases (gyro, accelerometer) from the calibration window, base is assumed not to rotate and
       * accelerate during it. Accelerometer bias needs the base orientation (quaternion x, y, z, w).
       */
      std::pair<vector3_t, vector3_t> computeImuBiases(const vector4_t& baseQuaternion) const;

      bool readImu(const sensor_msgs::msg::Imu& imu);

      bool readJointStates(const sensor_msgs::msg::JointState& jointStates);

      bool readContacts(const contact_msgs::msg::Contacts& contacts);

      void processContacts(const contact_msgs::msg::Contacts::ConstSharedPtr contacts);

      // Helper data
      size_t endEffectorNum_ = 0;
      std::vector<std::string> jointNames_;
      std::unordered_map<std::string, size_t> jointNameIndexMap_;
      std::unordered_map<std::string, size_t> contactFrameNameIndexMap_;

      // Read once in on_configure, parameter can be changed at runtime
      bool publishJointEstimates_ = false;
      
      // Data from sensors / observations (IMU, joint states, contact sensors)
      bool jointStateReceived_ = false;
      bool imuReceived_ = false;
      bool contactsReceived_ = false;
      ocs2::vector_t jointPositions_;
      ocs2::vector_t jointVelocities_;
      ocs2::vector_t jointTorques_;
      vector3_t angularVelocity_;
      vector3_t linearAcceleration_;
      std::vector<std::pair<int, bool>> contactFlags_;

      // Messages are checked in buffers, previous data is kept if they are invalid
      ocs2::vector_t jointPositionsBuffer_;
      ocs2::vector_t jointVelocitiesBuffer_;
      ocs2::vector_t jointTorquesBuffer_;
      std::vector<bool> jointUpdatedBuffer_;
      std::vector<bool> contactUpdatedBuffer_;

      // Starting base state from /initial_odometry, kept for next activations
      vector3_t initialBasePosition_ = vector3_t::Zero();
      vector4_t initialBaseQuaternion_ = vector4_t(0.0, 0.0, 0.0, 1.0);
      vector3_t initialBaseLinearVelocityWorld_ = vector3_t::Zero();
      bool initialOdometryAvailable_ = false;
      bool newInitialOdometry_ = false;

      // IMU measurements received before activation, used for the initial IMU biases
      struct ImuSample
      {
        rclcpp::Time time;
        vector3_t angularVelocity;
        vector3_t linearAcceleration;
      };
      std::deque<ImuSample> imuCalibrationSamples_;
      rclcpp::Duration imuCalibrationDuration_ = rclcpp::Duration(0, 0);

      // Estimator was updated since initialization from starting base pose
      bool estimateAvailable_ = false;

      // Estimator times are relative to this stamp (IMU measurement from activation)
      rclcpp::Time estimatorStartTime_;
      
      // Legged state estimator
      bool estimatorRunning_ = false;
      std::unique_ptr<legged_state_estimator::LeggedStateEstimator> leggedStateEstimator_;

      // Timer for sensor timeouts, estimator is updated for every IMU message
      ocs2::scalar_t estimatorDuration_ = 0.0;
      rclcpp::TimerBase::SharedPtr estimatorLoopTimer_;

      // Sensor/observation subscribers
      rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr initialOdometrySubscriber_;
      rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointStateSubscriber_;
      rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSubscriber_;
      rclcpp::Subscription<contact_msgs::msg::Contacts>::SharedPtr contactsSubscriber_;

      // Base (pose and twist) and joint state estimation publishers
      std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<
        nav_msgs::msg::Odometry>> odometryEstimatePublisher_;
      std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<
        sensor_msgs::msg::JointState>> jointStatesEstimatePublisher_;

      // Preallocated in on_configure, only stamps and values change when publishing
      nav_msgs::msg::Odometry odometryMessage_;
      sensor_msgs::msg::JointState jointStatesMessage_;

      // Last time robot sensor messages were recived
      rclcpp::Time lastJointStateTime_;
      rclcpp::Time lastImuTime_;
      rclcpp::Time lastContactFlagsTime_;
      rclcpp::Time lastContactsReceiveTime_;
      
      // Maximum age of IMU and joint states messages, estimator is deactivated after that
      rclcpp::Duration imuTimeout_ = rclcpp::Duration(0, 0);
      rclcpp::Duration jointStateTimeout_ = rclcpp::Duration(0, 0);

      // Maximum time without contacts messages (receive time), estimator is deactivated after that
      rclcpp::Duration contactsTimeout_ = rclcpp::Duration(0, 0);
  };  
} // namespace legged_state_estimator_ros2


#endif