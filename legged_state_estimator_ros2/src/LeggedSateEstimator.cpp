#include <legged_state_estimator_ros2/LeggedStateEstimator.hpp>

#include <pinocchio/parsers/urdf.hpp>

#include <tf2_eigen/tf2_eigen.hpp>

namespace legged_state_estimator_ros2
{
  using namespace ocs2;
  using namespace legged_state_estimator;

  LeggedStateEstimator::LeggedStateEstimator():
    LifecycleNode("legged_state_estimator", 
      NodeOptions().use_intra_process_comms(false))
  {
    // Config directory parameter
    this->declare_parameter("config_directory_path", "./");
    this->declare_parameter("urdf_path", "./");

    lastJointStateTime_ = this->get_clock()->now();
    lastImuTime_ = this->get_clock()->now();
    lastContactFlagsTime_ = this->get_clock()->now();

    estimatorRunning_ = false;

    RCLCPP_INFO(this->get_logger(), "Legged State Estimator in unconfigured state!");
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    LeggedStateEstimator::on_configure(const State& state)
  {
    const std::string configDirectoryPath = this->get_parameter("config_directory_path").as_string();

    LeggedStateEstimatorSettings estimatorSettings;
    try
    {
      estimatorSettings = loadLeggedStateEstimatorSettings(
        configDirectoryPath + "/legged_estimator.info", "legged_state_estimator", false);
    }
    catch(const std::exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "Error: Cannot find legged_estimator.info file!");
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    const std::string urdfFilePath = this->get_parameter("urdf_path").as_string();

    // Construct legged state estimator
    try
    {
      leggedStateEstimator_ = make_unique<legged_state_estimator::LeggedStateEstimator>(
        urdfFilePath, estimatorSettings);
    }
    catch(const std::exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "Error: Failed to intialize LeggedStateEstimator!");
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    /** 
     * Setup helper data structures
     */
    const auto& contactFrameIndicies = leggedStateEstimator_->getRobotModel().getContactFrames();

    if(contactFrameIndicies.size() != estimatorSettings.contact_frames.size())
    {
      RCLCPP_ERROR(this->get_logger(), "Error: contactFrameIndicies.size() != estimatorSettings.contact_frames.size()!");
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    for(size_t i = 0; i < estimatorSettings.contact_frames.size(); ++i)
    {
      contactFrameNameIndexMap_[estimatorSettings.contact_frames[i]] = i;
    }

    const auto model = pinocchio::urdf::buildModel(urdfFilePath);

    jointNames_ = model.names;

    // Remove "universe" and "root_joint" joints from joint names
    jointNames_.erase(std::remove(jointNames_.begin(), jointNames_.end(), "universe"), jointNames_.end());
    jointNames_.erase(std::remove(jointNames_.begin(), jointNames_.end(), "root_joint"), jointNames_.end()); 

    for(size_t i = 0; i < jointNames_.size(); ++i)
    {
      jointNameIndexMap_[jointNames_[i]] = i;
    }

    endEffectorNum_ = estimatorSettings.contact_frames.size();

    /**
     * Create subsciber and publishers
     */

    rclcpp::CallbackGroup::SharedPtr cb_group_not_executed = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);
    auto subscription_options = rclcpp::SubscriptionOptions();
    subscription_options.callback_group = cb_group_not_executed;
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos = qos.best_effort();
    
    // Joint states subscriber
    const std::string jointSatesTopic = "/joint_states";
    jointStateSubscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
      jointSatesTopic, qos, 
      std::bind(&LeggedStateEstimator::updateJointStates, this, std::placeholders::_1), subscription_options);
    
    // IMU subscriber
    const std::string imuTopic = "/imu";
    imuSubscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(imuTopic, qos, 
      std::bind(&LeggedStateEstimator::updateImu, this, std::placeholders::_1), subscription_options);
    
    // Contact flags subscriber
    const std::string contactFlagsTopic = "/contact_flags";
    contactsSubscriber_ = this->create_subscription<contact_msgs::msg::Contacts>(
      contactFlagsTopic, qos, 
      std::bind(&LeggedStateEstimator::updateContactFlags, this, std::placeholders::_1));

    // Base transform estimate publisher
    const std::string baseTransformTopic = "/base_transform_estimated";
    baseTransformEstimatePublisher_ = this->create_publisher<geometry_msgs::msg::TransformStamped>(
      baseTransformTopic, QoS(1).best_effort().keep_last(1));

    // Base twist estimate publisher
    const std::string baseTwistTopic = "/base_twist_estimated";
    baseTwistEstimatePublisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      baseTwistTopic, QoS(1).best_effort().keep_last(1));

    // Joint state estimate publisher
    const std::string jointStateTopic = "/joint_states_estimated";
    jointStatesEstimatePublisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
      jointStateTopic, QoS(1).best_effort().keep_last(1));

    /**
     * Create timer for joint trajectory commands
     */

    // Get duration 
    estimatorDuration_ = estimatorSettings.sample_time;

    // Estimator loop timer
    estimatorLoopTimer_ = rclcpp::create_timer(this, this->get_clock(), 
      rclcpp::Duration::from_seconds(estimatorDuration), 
      std::bind(&LeggedStateEstimator::sendStateEstimations, this));

    RCLCPP_INFO(this->get_logger(), "Legged State Estimator configured successfully!");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    LeggedStateEstimator::on_activate(const State& state)
  {
    leggedStateEstimator_->init(const vector3_t& base_pos, const vector4_t& base_quat,
      const vector3_t& base_lin_vel_world=vector3_t::Zero(),
      const vector3_t& imu_gyro_bias=vector3_t::Zero(),
      const vector3_t& imu_lin_accel_bias=vector3_t::Zero());

    estimatorRunning_ = true;

    RCLCPP_INFO(this->get_logger(), "Legged State Estimator activated successfully!");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    LeggedStateEstimator::on_deactivate(const State& state)
  {
    estimatorRunning_ = false;
    RCLCPP_INFO(this->get_logger(), "Legged State Estimator deactivated successfully!");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  void LeggedStateEstimator::updateCurrentSensorData()
  {
    rclcpp::MessageInfo msgInfo;

    // Joint states
    sensor_msgs::msg::JointState jointStates;
    if(jointStateSubscriber_->take(jointStates, msgInfo)) 
    {
      // Check if message has same amount of data
      if(jointStates.name.size() != jointStates.position.size() || 
         jointStates.name.size() != jointStates.velocity.size() ||
         jointStates.name.size() != jointNames_.size())
      {
        RCLCPP_ERROR(this->get_logger(), 
          "Ignored an invalid JointState message");
        return;
      }

      if(this->get_clock()->now() - lastJointStateTime_ > maxDurationBetweenMessages_)
      {
        RCLCPP_WARN(this->get_logger(), 
          "Time between two JointState messages was longer than maximum duration!");
      }

      lastJointStateTime_ = jointStates.header.stamp;

      jointPositions_ = vector_t(jointNames_.size());
      jointVelocities_ = vector_t(jointNames_.size());
      jointTorques_ = vector_t(jointNames_.size());

      for(size_t i = 0; i < jointStates.name.size(); ++i)
      {
        const size_t currentIndex = jointNameIndexMap_.at(jointStates.name[i]);
        jointPositions_[currentIndex] = jointStates.position[i];
        jointVelocities_[currentIndex] = jointStates.velocity[i];
        jointTorques_[currentIndex] = jointStates.torque[i];
      }
    }

    sensor_msgs::msg::Imu imuData;
    if(imuSubscriber_->take(imuData, msgInfo)) 
    {
      const auto& estimatorSettings = leggedStateEstimator_->getSettings();
      
      if(imuData.header.frame_id != estimatorSettings.imu_frame)
      {
        RCLCPP_ERROR(this->get_logger(), 
          "Ignored an invalid IMU message");
        return;
      }

      if(this->get_clock()->now() - lastImuTime_ > maxDurationBetweenMessages_)
      {
        RCLCPP_WARN(this->get_logger(), 
          "Time between two IMU messages was longer than maximum duration!");
      }

      lastImuTime_ = imuData.header.stamp;

      tf2::fromMsg(imuData.orientation, quaterion_);
      tf2::fromMsg(imuData.angular_velocity, angularVelocity_);
      tf2::fromMsg(imuData.linear_acceleration, linearAcceleration_);
    }

    contact_msgs::msg::Contacts contacts;
    if(contactsSubscriber_->take(contacts, msgInfo)) 
    {
      
      if(contactFlags->contacts.size() != endEffectorNum_)
      {
        RCLCPP_ERROR(this->get_logger(), 
          "Ignored an invalid Contacts message");
        return;
      }

      if(this->get_clock()->now() - lastContactFlagsTime_ > maxDurationBetweenMessages_)
      {
        RCLCPP_WARN(this->get_logger(), 
          "Time between two Contacts messages was longer than maximum duration!");
      }

      // TODO SPRAWDZ CZY TO NIE JEST TAK ZE KAZDY MA INNY TIMESTAMP
      lastContactFlagsTime_ = contactFlags->contacts[0].header.stamp;

      contactFlags_.clear();
      contactFlags_.reserve(endEffectorNum_);

      for(size_t i = 0; i < endEffectorNum_; ++i)
      {
        const auto& currentContactMessage = contactFlags->contacts[i];
        const size_t currentIndex = contactFrameNameIndexMap_[
          currentContactMessage.header.frame_id];

        contactFlags_.push_back(std::pair<int, bool>(currentIndex, 
          currentContactMessage.contact));
      }
    }
  }

  void LeggedStateEstimator::sendStateEstimations()
  {
    if(!estimatorRunning_) return;

    // Update current sensor data
    this->updateCurrentSensorData();

    const bool jointsReady = jointPositions_.size() == jointNames_.size();
    const bool imuReady = (quaterion_.toRotationMatrix() - quaterion_t().toRotationMatrix()).norm() > 1e-3;
    const bool contactsReady = contactFlags_.size() == endEffectorNum_;

    if(jointsReady && imuReady && contactsReady)
    {
      const auto currentTime = this->get_clock()->now();
      leggedStateEstimator_->update(angularVelocity_, linearAcceleration_, 
        jointPositions_, jointVelocities_, contactFlags_);

      const auto& estimatorSettings = leggedStateEstimator_->getSettings();
      
      // Base transform estimate
      const vector3_t& basePosition = leggedStateEstimator_->getBasePositionEstimate();
      const vector4_t& quaterionCoeffs = leggedStateEstimator_->getBaseQuaternionEstimate();
      
      geometry_msgs::msg::TransformStamped baseTransform;

      baseTransform.header.frame_id = estimatorSettings.world_frame;
      baseTransform.header.stamp = currentTime;
      baseTransform.child_frame_id = estimatorSettings.base_frame;

      tf2::toMsg(basePosition, baseTransform.transform.translation);

      baseTransform.transform.rotation.x = quaterionCoeffs(0);
      baseTransform.transform.rotation.y = quaterionCoeffs(1);
      baseTransform.transform.rotation.z = quaterionCoeffs(2);
      baseTransform.transform.rotation.w = quaterionCoeffs(3);

      // Base twist estimate
      const vector3_t& baseLinearVelocity = 
        leggedStateEstimator_->getBaseLinearVelocityEstimateLocal();
      const vector3_t& baseAngularVelocity = 
        leggedStateEstimator_->getBaseAngularVelocityEstimateLocal();

      geometry_msgs::msg::TwistStamped baseTwist;
      baseTwist.header.frame_id = estimatorSettings.base_frame;
      baseTwist.header.stamp = currentTime;
      tf2::toMsg(baseLinearVelocity, baseTwist.twist.linear);
      tf2::toMsg(baseAngularVelocity, baseTwist.twist.angular);

      // Joint state estimates
      const vector_t& jointPositions = jointPositions_;
      const vector_t& jointVelocities = leggedStateEstimator_->getJointVelocityEstimate();
      const vector_t& jointTorques = leggedStateEstimator_->getJointTorqueEstimate();

      sensor_msgs::msg::JointState jointStates;

      jointStates.header.stamp = currentTime;

      const std::vector<scalar_t> positions(jointPositions.data(), 
          jointPositions.data() + jointPositions.size());

      const std::vector<scalar_t> velocities(jointVelocities.data(), 
          jointVelocities.data() + jointVelocities.size());

      const std::vector<scalar_t> torques(jointTorques.data(), 
          jointTorques.data() + jointTorques.size());

      jointStates.name = jointNames_;
      jointStates.position = positions;
      jointStates.velocity = velocities;
      jointStates.torque = torques;

      // Publish
      baseTransformEstimatePublisher_->publish(baseTransform);
      baseTwistEstimatePublisher_->publish(baseTwist);
      jointStatesEstimatePublisher_->publish(jointStates);
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), 
          "Joints, IMU or contact flags are not ready!");
      return;
    }
  }
} // legged_state_estimator_ros2