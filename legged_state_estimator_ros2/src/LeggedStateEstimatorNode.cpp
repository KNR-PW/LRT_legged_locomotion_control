#include <legged_state_estimator_ros2/LeggedStateEstimatorNode.hpp>

#include <pinocchio/parsers/urdf.hpp>

#include <tf2_eigen/tf2_eigen.hpp>

namespace legged_state_estimator_ros2
{
  using namespace ocs2;
  using namespace legged_state_estimator;
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  LeggedStateEstimatorNode::LeggedStateEstimatorNode():
    LifecycleNode("legged_state_estimator", 
      rclcpp::NodeOptions().use_intra_process_comms(false))
  {
    // Config directory parameter
    this->declare_parameter("config_directory_path", "./");
    this->declare_parameter("urdf_path", "./");
    this->declare_parameter("publish_joint_estimates", false);

    lastJointStateTime_ = this->get_clock()->now();
    lastImuTime_ = this->get_clock()->now();
    lastContactFlagsTime_ = this->get_clock()->now();

    estimatorRunning_ = false;

    RCLCPP_INFO(this->get_logger(), "Legged State Estimator in unconfigured state!");
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    LeggedStateEstimatorNode::on_configure(const rclcpp_lifecycle::State& state)
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
      leggedStateEstimator_ = std::make_unique<LeggedStateEstimator>(urdfFilePath, estimatorSettings);
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

    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdfFilePath, model);

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

    // Start base transform subscriber
    const std::string startBaseTransformTopic = "/initial_base_transform";
    startBaseTransformSubscriber_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(
      startBaseTransformTopic, rclcpp::QoS(1).reliable(), 
      [](const geometry_msgs::msg::TransformStamped::ConstSharedPtr) {}, subscription_options);
    
    // Joint states subscriber
    const std::string jointSatesTopic = "/joint_states";
    jointStateSubscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
      jointSatesTopic, qos, 
      [](const sensor_msgs::msg::JointState::ConstSharedPtr) {}, subscription_options);
    
    // IMU subscriber
    const std::string imuTopic = "/imu";
    imuSubscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(imuTopic, qos, 
      [](const sensor_msgs::msg::Imu::ConstSharedPtr) {}, subscription_options);
    
    // Contact flags subscriber
    if(!estimatorSettings.use_contact_estimator)
    {
      const std::string contactFlagsTopic = "/contact_flags";
      contactsSubscriber_ = this->create_subscription<contact_msgs::msg::Contacts>(
        contactFlagsTopic, qos, 
        [](const contact_msgs::msg::Contacts::ConstSharedPtr) {}, subscription_options);
    }

    // Base transform estimate publisher
    const std::string baseTransformTopic = "/base_transform_estimated";
    baseTransformEstimatePublisher_ = this->create_publisher<geometry_msgs::msg::TransformStamped>(
      baseTransformTopic, rclcpp::QoS(1).best_effort().keep_last(1));

    // Base twist estimate publisher
    const std::string baseTwistTopic = "/base_twist_estimated";
    baseTwistEstimatePublisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      baseTwistTopic, rclcpp::QoS(1).best_effort().keep_last(1));

    // Joint state estimate publisher

    if(this->get_parameter("publish_joint_estimates").as_bool())
    {
      const std::string jointStateTopic = "/joint_states_estimated";
      jointStatesEstimatePublisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
        jointStateTopic, rclcpp::QoS(1).best_effort().keep_last(1));
    }

    /**
     * Create timer for joint trajectory commands
     */

    // Get duration 
    estimatorDuration_ = estimatorSettings.sampling_time;

    // Estimator loop timer
    estimatorLoopTimer_ = rclcpp::create_timer(this, this->get_clock(), 
      rclcpp::Duration::from_seconds(estimatorDuration_), 
      std::bind(&LeggedStateEstimatorNode::sendStateEstimations, this));

    RCLCPP_INFO(this->get_logger(), "Legged State Estimator configured successfully!");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    LeggedStateEstimatorNode::on_activate(const rclcpp_lifecycle::State& state)
  {

    rclcpp::MessageInfo msgInfo;

    // Start base transform message
    geometry_msgs::msg::TransformStamped startBaseTransform;
    if(startBaseTransformSubscriber_->take(startBaseTransform, msgInfo)) 
    {
      vector3_t startPosition;
      quaternion_t startQuaternion;
      tf2::fromMsg(startBaseTransform.transform.translation, startPosition);
      tf2::fromMsg(startBaseTransform.transform.rotation, startQuaternion);
      const vector4_t quaternionVector = startQuaternion.coeffs(); 

      leggedStateEstimator_->init(startPosition, quaternionVector);
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Could not get starting base transform, failed to activate!");
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }

    estimatorRunning_ = true;
    RCLCPP_INFO(this->get_logger(), "Legged State Estimator activated successfully!");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    LeggedStateEstimatorNode::on_deactivate(const rclcpp_lifecycle::State& state)
  {
    estimatorRunning_ = false;
    
    jointPositions_ = vector_t();
    jointVelocities_ = vector_t();
    jointTorques_ = vector_t();
    quaterion_ = quaternion_t(1.0, 0.0, 0.0, 0.0);
    angularVelocity_.setZero();
    linearAcceleration_.setZero();
    contactFlags_.clear();

    RCLCPP_INFO(this->get_logger(), "Legged State Estimator deactivated successfully!");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void LeggedStateEstimatorNode::updateCurrentSensorData()
  {
    const auto& estimatorSettings = leggedStateEstimator_->getSettings();
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
        jointTorques_[currentIndex] = jointStates.effort[i];
      }
    }

    sensor_msgs::msg::Imu imuData;
    if(imuSubscriber_->take(imuData, msgInfo)) 
    {
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

    if(estimatorSettings.use_contact_estimator) return;

    contact_msgs::msg::Contacts contacts;
    if(contactsSubscriber_->take(contacts, msgInfo)) 
    {
      
      if(contacts.contacts.size() != endEffectorNum_)
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
      lastContactFlagsTime_ = contacts.contacts[0].header.stamp;

      contactFlags_.clear();
      contactFlags_.reserve(endEffectorNum_);

      for(size_t i = 0; i < endEffectorNum_; ++i)
      {
        const auto& currentContactMessage = contacts.contacts[i];
        const size_t currentIndex = contactFrameNameIndexMap_[
          currentContactMessage.header.frame_id];

        contactFlags_.push_back(std::pair<int, bool>(currentIndex, 
          currentContactMessage.contact));
      }
    }
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void LeggedStateEstimatorNode::sendStateEstimations()
  {
    if(!estimatorRunning_) return;

    const auto& estimatorSettings = leggedStateEstimator_->getSettings();

    // Update current sensor data
    this->updateCurrentSensorData();

    const bool jointsReady = jointPositions_.size() == jointNames_.size();
    const bool imuReady = (quaterion_.toRotationMatrix() - quaternion_t().toRotationMatrix()).norm() > 1e-3;
    const bool contactsReady = contactFlags_.size() == endEffectorNum_ || estimatorSettings.use_contact_estimator;

    if(jointsReady && imuReady && contactsReady)
    {
      const auto currentTime = this->get_clock()->now();

      if(estimatorSettings.use_contact_estimator)
      {
        leggedStateEstimator_->update(angularVelocity_, linearAcceleration_, 
          jointPositions_, jointVelocities_, jointTorques_);
      }
      else
      {
        leggedStateEstimator_->update(angularVelocity_, linearAcceleration_, 
          jointPositions_, jointVelocities_, jointTorques_, contactFlags_);
      }

      const auto& estimatorSettings = leggedStateEstimator_->getSettings();
      
      // Base transform estimate
      const vector3_t& basePosition = leggedStateEstimator_->getBasePositionEstimate();
      const vector4_t& quaternionCoeffs = leggedStateEstimator_->getBaseQuaternionEstimate();
      
      geometry_msgs::msg::TransformStamped baseTransform;

      baseTransform.header.frame_id = estimatorSettings.world_frame;
      baseTransform.header.stamp = currentTime;
      baseTransform.child_frame_id = estimatorSettings.base_frame;

      tf2::toMsg(basePosition, baseTransform.transform.translation);

      baseTransform.transform.rotation.x = quaternionCoeffs(0);
      baseTransform.transform.rotation.y = quaternionCoeffs(1);
      baseTransform.transform.rotation.z = quaternionCoeffs(2);
      baseTransform.transform.rotation.w = quaternionCoeffs(3);

      baseTransformEstimatePublisher_->publish(baseTransform);

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

      baseTwistEstimatePublisher_->publish(baseTwist);

      if(this->get_parameter("publish_joint_estimates").as_bool())
      {
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
        jointStates.position = std::move(positions);
        jointStates.velocity = std::move(velocities);
        jointStates.effort = std::move(torques);

        jointStatesEstimatePublisher_->publish(jointStates);
      }
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), 
        "Joints, IMU or contact flags are not ready!");
      return;
    }
  }
} // legged_state_estimator_ros2