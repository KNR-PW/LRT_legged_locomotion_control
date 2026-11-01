#include <legged_state_estimator_ros2/LeggedStateEstimatorNode.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>

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

    // Maximum age of IMU and joint states messages [s], estimator is deactivated after that
    this->declare_parameter("imu_timeout", 0.1);
    this->declare_parameter("joint_state_timeout", 0.1);

    // Maximum time without contacts messages [s] (only without contact estimator), estimator is
    // deactivated after that. Receive time is used, contact stamps are often not set
    this->declare_parameter("contacts_timeout", 0.1);

    // Duration of IMU data [s] received before activation, used for initial IMU biases.
    // Base must not rotate and accelerate during it
    this->declare_parameter("imu_bias_calibration_duration", 1.0);

    // Same clock type as node clock, times from different clocks can not be compared
    const rclcpp::Time zeroTime(0, 0, this->get_clock()->get_clock_type());
    lastJointStateTime_ = zeroTime;
    lastImuTime_ = zeroTime;
    lastContactFlagsTime_ = zeroTime;
    lastContactsReceiveTime_ = zeroTime;
    estimatorStartTime_ = zeroTime;

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
      RCLCPP_ERROR(this->get_logger(), "Error while loading legged_estimator.info file: %s", e.what());
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    const scalar_t imuTimeout = this->get_parameter("imu_timeout").as_double();
    const scalar_t jointStateTimeout = this->get_parameter("joint_state_timeout").as_double();
    const scalar_t contactsTimeout = this->get_parameter("contacts_timeout").as_double();
    const scalar_t imuCalibrationDuration = this->get_parameter("imu_bias_calibration_duration").as_double();

    if(!(imuTimeout > 0.0) || !(jointStateTimeout > 0.0) || !(contactsTimeout > 0.0) ||
       !(imuCalibrationDuration > 0.0))
    {
      RCLCPP_ERROR(this->get_logger(),
        "Parameters imu_timeout, joint_state_timeout, contacts_timeout and "
        "imu_bias_calibration_duration have to be positive!");
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    imuTimeout_ = rclcpp::Duration::from_seconds(imuTimeout);
    jointStateTimeout_ = rclcpp::Duration::from_seconds(jointStateTimeout);
    contactsTimeout_ = rclcpp::Duration::from_seconds(contactsTimeout);
    imuCalibrationDuration_ = rclcpp::Duration::from_seconds(imuCalibrationDuration);

    // Read once, joint estimates publisher exists only when it is enabled here
    publishJointEstimates_ = this->get_parameter("publish_joint_estimates").as_bool();

    const std::string urdfFilePath = this->get_parameter("urdf_path").as_string();

    // Construct legged state estimator
    try
    {
      leggedStateEstimator_ = std::make_unique<LeggedStateEstimator>(urdfFilePath, estimatorSettings);
    }
    catch(const std::exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "Error while creating LeggedStateEstimator: %s", e.what());
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    // New estimator, it has to be initialized from initial odometry received after configuration
    estimateAvailable_ = false;
    initialOdometryAvailable_ = false;
    newInitialOdometry_ = false;
    imuCalibrationSamples_.clear();

    /** 
     * Setup helper data structures
     */
    const auto& contactFrameIndicies = leggedStateEstimator_->getRobotModel().getContactFrames();

    if(contactFrameIndicies.size() != estimatorSettings.contact_frames.size())
    {
      RCLCPP_ERROR(this->get_logger(), "Error: contactFrameIndicies.size() != estimatorSettings.contact_frames.size()!");
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    // Names from previous configuration can be different
    contactFrameNameIndexMap_.clear();

    for(size_t i = 0; i < estimatorSettings.contact_frames.size(); ++i)
    {
      contactFrameNameIndexMap_[estimatorSettings.contact_frames[i]] = i;
    }

    // Joint names from the estimator model (URDF is parsed once), in order of the joint vectors
    jointNames_ = leggedStateEstimator_->getRobotModel().getJointNames();

    // Joint vectors are indexed by joint names, so every joint has to have one degree of freedom
    if(jointNames_.size() != static_cast<size_t>(leggedStateEstimator_->getRobotModel().nJ()))
    {
      RCLCPP_ERROR(this->get_logger(), "Error: Only joints with one degree of freedom are supported!");
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    jointNameIndexMap_.clear();

    for(size_t i = 0; i < jointNames_.size(); ++i)
    {
      jointNameIndexMap_[jointNames_[i]] = i;
    }

    endEffectorNum_ = estimatorSettings.contact_frames.size();

    // Sensor data and buffers, sized once
    const size_t jointNum = jointNames_.size();
    jointPositions_ = vector_t::Zero(jointNum);
    jointVelocities_ = vector_t::Zero(jointNum);
    jointTorques_ = vector_t::Zero(jointNum);
    jointPositionsBuffer_ = vector_t::Zero(jointNum);
    jointVelocitiesBuffer_ = vector_t::Zero(jointNum);
    jointTorquesBuffer_ = vector_t::Zero(jointNum);
    jointUpdatedBuffer_.assign(jointNum, false);
    contactUpdatedBuffer_.assign(endEffectorNum_, false);
    contactFlags_.clear();

    /**
     * Create subsciber and publishers
     */

    rclcpp::CallbackGroup::SharedPtr cbGroupNotExecuted = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);
    auto subscriptionOptions = rclcpp::SubscriptionOptions();
    subscriptionOptions.callback_group = cbGroupNotExecuted;

    // Initial odometry subscriber (latched), starting base pose and velocity
    const std::string initialOdometryTopic = "/initial_odometry";
    initialOdometrySubscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      initialOdometryTopic, rclcpp::QoS(1).reliable().transient_local(),
      [](const nav_msgs::msg::Odometry::ConstSharedPtr) {}, subscriptionOptions);

    // Joint states subscriber, estimator is corrected with every joint states message.
    // Queue keeps messages received while other callback is running, so none is lost
    const std::string jointSatesTopic = "/joint_states";
    jointStateSubscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
      jointSatesTopic, rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
      std::bind(&LeggedStateEstimatorNode::processJointStates, this, std::placeholders::_1));
    
    // IMU subscriber, estimator is propagated with every IMU message.
    // Before activation messages are collected for the initial IMU biases
    const std::string imuTopic = "/imu";
    imuSubscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(imuTopic, 
      rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
      std::bind(&LeggedStateEstimatorNode::processImu, this, std::placeholders::_1));
    
    // Contact flags subscriber, contact states are given to the estimator for every message
    if(!estimatorSettings.use_contact_estimator)
    {
      const std::string contactFlagsTopic = "/contacts";
      contactsSubscriber_ = this->create_subscription<contact_msgs::msg::Contacts>(
        contactFlagsTopic, rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(), 
        std::bind(&LeggedStateEstimatorNode::processContacts, this, std::placeholders::_1));
    }
    else
    {
      contactsSubscriber_.reset();
    }

    // Base pose and twist estimate publisher (read by MPC and WBC controllers)
    const std::string odometryTopic = "/odom";
    odometryEstimatePublisher_ = this->create_publisher<nav_msgs::msg::Odometry>(
      odometryTopic, rclcpp::QoS(1).best_effort());

    // Joint state estimate publisher

    if(publishJointEstimates_)
    {
      const std::string jointStateTopic = "/joint_states_estimated";
      jointStatesEstimatePublisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
        jointStateTopic, rclcpp::QoS(1).best_effort());
    }
    else
    {
      jointStatesEstimatePublisher_.reset();
    }

    // Estimate messages, only stamps and values change when they are published
    odometryMessage_ = nav_msgs::msg::Odometry();
    odometryMessage_.header.frame_id = estimatorSettings.world_frame;
    odometryMessage_.child_frame_id = estimatorSettings.base_frame;

    jointStatesMessage_ = sensor_msgs::msg::JointState();
    jointStatesMessage_.name = jointNames_;
    jointStatesMessage_.position.resize(jointNum);
    jointStatesMessage_.velocity.resize(jointNum);
    jointStatesMessage_.effort.resize(jointNum);

    /**
     * Create timer for sensor timeouts
     */

    // Get duration 
    estimatorDuration_ = estimatorSettings.sampling_time;

    // Sensor timeout timer, estimator itself is updated for every IMU and joint states message
    estimatorLoopTimer_ = rclcpp::create_timer(this, this->get_clock(), 
      rclcpp::Duration::from_seconds(estimatorDuration_), 
      std::bind(&LeggedStateEstimatorNode::checkSensorTimeouts, this));

    RCLCPP_INFO(this->get_logger(), "Legged State Estimator configured successfully!");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    LeggedStateEstimatorNode::on_activate(const rclcpp_lifecycle::State& state)
  {
    if(!waitForInitialData())
    {
      RCLCPP_ERROR(this->get_logger(), "No recent%s, failed to activate!",
        getMissingInitialData(this->get_clock()->now()).c_str());
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }

    try
    {
      const bool fromInitialOdometry = newInitialOdometry_ || !estimateAvailable_;

      vector3_t basePosition;
      vector4_t baseQuaternion;
      vector3_t baseLinearVelocityWorld;
      if(fromInitialOdometry)
      {
        // Base pose and velocity from initial odometry
        basePosition = initialBasePosition_;
        baseQuaternion = initialBaseQuaternion_;
        baseLinearVelocityWorld = initialBaseLinearVelocityWorld_;
      }
      else
      {
        // Reactivation without new initial odometry: continue from last estimate,
        // robot is assumed to stand still
        basePosition = leggedStateEstimator_->getBasePositionEstimate();
        baseQuaternion = leggedStateEstimator_->getBaseQuaternionEstimate();
        baseLinearVelocityWorld = vector3_t::Zero();
      }

      // IMU biases from measurements received before activation
      const auto imuBiases = computeImuBiases(baseQuaternion);
      leggedStateEstimator_->init(basePosition, baseQuaternion, baseLinearVelocityWorld,
        imuBiases.first, imuBiases.second);

      RCLCPP_INFO(this->get_logger(),
        "Initial IMU biases, gyro: [%f, %f, %f], accelerometer: [%f, %f, %f]",
        imuBiases.first.x(), imuBiases.first.y(), imuBiases.first.z(),
        imuBiases.second.x(), imuBiases.second.y(), imuBiases.second.z());

      if(fromInitialOdometry)
      {
        newInitialOdometry_ = false;
        estimateAvailable_ = false;
      }
    }
    catch(const std::exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "Error while initializing LeggedStateEstimator: %s", e.what());
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    // Estimator times are relative to the IMU measurement from activation, which
    // sets the time of the estimate and is held until the next measurement
    estimatorStartTime_ = lastImuTime_;
    leggedStateEstimator_->updateImu(angularVelocity_, linearAcceleration_, 
      toEstimatorTime(lastImuTime_));

    // Contact states from activation are used until next contacts message
    if(!leggedStateEstimator_->getSettings().use_contact_estimator)
    {
      leggedStateEstimator_->updateContacts(contactFlags_, toEstimatorTime(lastContactFlagsTime_));
    }

    odometryEstimatePublisher_->on_activate();
    if(jointStatesEstimatePublisher_)
    {
      jointStatesEstimatePublisher_->on_activate();
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
    odometryEstimatePublisher_->on_deactivate();
    if(jointStatesEstimatePublisher_)
    {
      jointStatesEstimatePublisher_->on_deactivate();
    }
    
    // Sensor data has to be received again during next activation
    jointStateReceived_ = false;
    imuReceived_ = false;
    contactsReceived_ = false;

    // IMU biases for next activation come from measurements after this deactivation
    imuCalibrationSamples_.clear();

    RCLCPP_INFO(this->get_logger(), "Legged State Estimator deactivated successfully!");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  bool LeggedStateEstimatorNode::waitForInitialData()
  {
    // Sensor data has to be received during this activation
    jointStateReceived_ = false;
    imuReceived_ = false;
    contactsReceived_ = false;

    const bool useContactEstimator = leggedStateEstimator_->getSettings().use_contact_estimator;

    // Steady clock, so waiting also ends when ROS time is not running.
    // Activation right after configuration also waits for IMU bias calibration data
    const auto maxWaitDuration = std::chrono::seconds(1) +
      std::chrono::nanoseconds(imuCalibrationDuration_.nanoseconds());
    const auto startTime = std::chrono::steady_clock::now();

    while(rclcpp::ok())
    {
      updateInitialOdometry();

      // IMU, joint states and contacts callbacks are not executed during activation, newest messages are used
      rclcpp::MessageInfo msgInfo;

      if(!useContactEstimator)
      {
        contact_msgs::msg::Contacts contacts;
        while(contactsSubscriber_->take(contacts, msgInfo))
        {
          readContacts(contacts);
        }
      }

      sensor_msgs::msg::JointState jointStates;
      while(jointStateSubscriber_->take(jointStates, msgInfo))
      {
        readJointStates(jointStates);
      }

      sensor_msgs::msg::Imu imu;
      while(imuSubscriber_->take(imu, msgInfo))
      {
        if(readImu(imu))
        {
          addImuCalibrationSample();
        }
      }

      if(getMissingInitialData(this->get_clock()->now()).empty())
      {
        return true;
      }

      if(std::chrono::steady_clock::now() - startTime > maxWaitDuration)
      {
        return false;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    return false;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  std::string LeggedStateEstimatorNode::getMissingInitialData(const rclcpp::Time& currentTime) const
  {
    std::string missingData;

    // Initial odometry is needed until estimator was updated
    if(!initialOdometryAvailable_ && !estimateAvailable_)
    {
      missingData += " initial odometry";
    }

    if(!isJointStateFresh(currentTime))
    {
      missingData += " joint states";
    }

    if(!isImuFresh(currentTime))
    {
      missingData += " IMU data";
    }

    if(!isImuCalibrationReady())
    {
      missingData += " IMU bias calibration data";
    }

    if(!leggedStateEstimator_->getSettings().use_contact_estimator && !isContactsFresh(currentTime))
    {
      missingData += " contacts";
    }

    return missingData;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  bool LeggedStateEstimatorNode::isImuFresh(const rclcpp::Time& currentTime) const
  {
    return imuReceived_ && (currentTime - lastImuTime_) <= imuTimeout_;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  bool LeggedStateEstimatorNode::isJointStateFresh(const rclcpp::Time& currentTime) const
  {
    return jointStateReceived_ && (currentTime - lastJointStateTime_) <= jointStateTimeout_;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  bool LeggedStateEstimatorNode::isContactsFresh(const rclcpp::Time& currentTime) const
  {
    return contactsReceived_ && (currentTime - lastContactsReceiveTime_) <= contactsTimeout_;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void LeggedStateEstimatorNode::updateInitialOdometry()
  {
    rclcpp::MessageInfo msgInfo;

    // Initial odometry message
    nav_msgs::msg::Odometry initialOdometry;
    if(!initialOdometrySubscriber_->take(initialOdometry, msgInfo)) return;

    const auto& estimatorSettings = leggedStateEstimator_->getSettings();

    if(initialOdometry.header.frame_id != estimatorSettings.world_frame ||
       initialOdometry.child_frame_id != estimatorSettings.base_frame)
    {
      RCLCPP_ERROR(this->get_logger(),
        "Ignored initial odometry %s -> %s, expected %s -> %s",
        initialOdometry.header.frame_id.c_str(), initialOdometry.child_frame_id.c_str(),
        estimatorSettings.world_frame.c_str(), estimatorSettings.base_frame.c_str());
      return;
    }

    vector3_t startPosition;
    quaternion_t startQuaternion;
    vector3_t startLinearVelocityLocal;
    tf2::fromMsg(initialOdometry.pose.pose.position, startPosition);
    tf2::fromMsg(initialOdometry.pose.pose.orientation, startQuaternion);
    tf2::fromMsg(initialOdometry.twist.twist.linear, startLinearVelocityLocal);

    // Zero quaternion can not be normalized
    if(!startPosition.allFinite() || !startQuaternion.coeffs().allFinite() ||
       !startLinearVelocityLocal.allFinite() || startQuaternion.norm() < 1e-6)
    {
      RCLCPP_ERROR(this->get_logger(),
        "Ignored initial odometry with non-finite values or zero quaternion");
      return;
    }

    initialBasePosition_ = startPosition;

    // Estimator expects unit quaternion (x, y, z, w)
    const quaternion_t startUnitQuaternion = startQuaternion.normalized();
    initialBaseQuaternion_ = startUnitQuaternion.coeffs();

    // Odometry twist is expressed in child (base) frame, estimator takes velocity in world frame
    initialBaseLinearVelocityWorld_ = startUnitQuaternion.toRotationMatrix() * startLinearVelocityLocal;

    initialOdometryAvailable_ = true;
    newInitialOdometry_ = true;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void LeggedStateEstimatorNode::addImuCalibrationSample()
  {
    // Time jumped back (e.g. simulation reset), older measurements are not used
    if(!imuCalibrationSamples_.empty() && lastImuTime_ <= imuCalibrationSamples_.back().time)
    {
      imuCalibrationSamples_.clear();
    }

    imuCalibrationSamples_.push_back({lastImuTime_, angularVelocity_, linearAcceleration_});

    // Keep only the newest measurements that still span the calibration duration
    while(imuCalibrationSamples_.size() > 1 &&
          imuCalibrationSamples_.back().time - imuCalibrationSamples_[1].time >= imuCalibrationDuration_)
    {
      imuCalibrationSamples_.pop_front();
    }
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  bool LeggedStateEstimatorNode::isImuCalibrationReady() const
  {
    return imuCalibrationSamples_.size() > 1 &&
      imuCalibrationSamples_.back().time - imuCalibrationSamples_.front().time >= imuCalibrationDuration_;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  std::pair<vector3_t, vector3_t> LeggedStateEstimatorNode::computeImuBiases(
    const vector4_t& baseQuaternion) const
  {
    vector3_t meanAngularVelocity = vector3_t::Zero();
    vector3_t meanLinearAcceleration = vector3_t::Zero();
    for(const auto& sample: imuCalibrationSamples_)
    {
      meanAngularVelocity += sample.angularVelocity;
      meanLinearAcceleration += sample.linearAcceleration;
    }
    meanAngularVelocity /= static_cast<scalar_t>(imuCalibrationSamples_.size());
    meanLinearAcceleration /= static_cast<scalar_t>(imuCalibrationSamples_.size());

    // Without rotation gyro measures only its bias
    const vector3_t gyroBias = meanAngularVelocity;

    // Without acceleration accelerometer measures bias minus gravity (InEKF: a - bias = -R^T * g),
    // R is IMU orientation in world, gravity is the same as in InEKF
    const vector3_t gravity(0.0, 0.0, -9.81);
    const matrix3_t baseRotation = quaternion_t(baseQuaternion).toRotationMatrix();
    const matrix3_t imuRotation = baseRotation *
      leggedStateEstimator_->getRobotModel().getImuPlacementInBase().rotation();
    const vector3_t accelerometerBias = meanLinearAcceleration + imuRotation.transpose() * gravity;

    return {gyroBias, accelerometerBias};
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  bool LeggedStateEstimatorNode::readImu(const sensor_msgs::msg::Imu& imu)
  {
    const auto& estimatorSettings = leggedStateEstimator_->getSettings();

    if(imu.header.frame_id != estimatorSettings.imu_frame)
    {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Ignored IMU message with frame %s, expected %s",
        imu.header.frame_id.c_str(), estimatorSettings.imu_frame.c_str());
      return false;
    }

    const rclcpp::Time imuTime = imu.header.stamp;

    // Integration time step comes from stamps, they have to increase
    if(imuReceived_ && imuTime <= lastImuTime_)
    {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Ignored IMU message not newer than previous one");
      return false;
    }

    vector3_t angularVelocity;
    vector3_t linearAcceleration;
    tf2::fromMsg(imu.angular_velocity, angularVelocity);
    tf2::fromMsg(imu.linear_acceleration, linearAcceleration);

    // Non-finite values would stay in estimator state for good
    if(!angularVelocity.allFinite() || !linearAcceleration.allFinite())
    {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Ignored IMU message with non-finite values");
      return false;
    }

    angularVelocity_ = angularVelocity;
    linearAcceleration_ = linearAcceleration;

    lastImuTime_ = imuTime;
    imuReceived_ = true;
    return true;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  bool LeggedStateEstimatorNode::readJointStates(const sensor_msgs::msg::JointState& jointStates)
  {
    const rclcpp::Time jointTime = jointStates.header.stamp;

    // Time step of joint filters comes from stamps, they have to increase
    if(jointStateReceived_ && jointTime <= lastJointStateTime_)
    {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Ignored JointState message not newer than previous one");
      return false;
    }

    // Check if message has same amount of data
    if(jointStates.name.size() != jointStates.position.size() ||
       jointStates.name.size() != jointStates.velocity.size() ||
       jointStates.name.size() != jointStates.effort.size() ||
       jointStates.name.size() != jointNames_.size())
    {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Ignored an invalid JointState message");
      return false;
    }

    std::fill(jointUpdatedBuffer_.begin(), jointUpdatedBuffer_.end(), false);

    for(size_t i = 0; i < jointStates.name.size(); ++i)
    {
      const auto jointIndexIt = jointNameIndexMap_.find(jointStates.name[i]);

      // Unknown or duplicated joint would leave some joint data unset
      if(jointIndexIt == jointNameIndexMap_.end() || jointUpdatedBuffer_[jointIndexIt->second])
      {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
          "Ignored JointState message with unknown or duplicated joint: %s",
          jointStates.name[i].c_str());
        return false;
      }

      const size_t currentIndex = jointIndexIt->second;
      jointPositionsBuffer_[currentIndex] = jointStates.position[i];
      jointVelocitiesBuffer_[currentIndex] = jointStates.velocity[i];
      jointTorquesBuffer_[currentIndex] = jointStates.effort[i];
      jointUpdatedBuffer_[currentIndex] = true;
    }

    // Non-finite values would stay in estimator state for good
    if(!jointPositionsBuffer_.allFinite() || !jointVelocitiesBuffer_.allFinite() ||
       !jointTorquesBuffer_.allFinite())
    {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Ignored JointState message with non-finite values");
      return false;
    }

    // Swap keeps both vectors allocated
    jointPositions_.swap(jointPositionsBuffer_);
    jointVelocities_.swap(jointVelocitiesBuffer_);
    jointTorques_.swap(jointTorquesBuffer_);

    lastJointStateTime_ = jointTime;
    jointStateReceived_ = true;
    return true;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  bool LeggedStateEstimatorNode::readContacts(const contact_msgs::msg::Contacts& contacts)
  {
    if(contacts.contacts.size() != endEffectorNum_)
    {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Ignored an invalid Contacts message");
      return false;
    }

    std::fill(contactUpdatedBuffer_.begin(), contactUpdatedBuffer_.end(), false);
    std::vector<std::pair<int, bool>> newContactFlags(endEffectorNum_);

    // Contacts message has no header, every contact has own stamp
    rclcpp::Time newestContactTime = lastContactFlagsTime_;

    for(const auto& currentContactMessage: contacts.contacts)
    {
      const auto contactIndexIt = contactFrameNameIndexMap_.find(
        currentContactMessage.header.frame_id);

      // Unknown or duplicated frame would leave contact state of some leg unchanged
      if(contactIndexIt == contactFrameNameIndexMap_.end() ||
         contactUpdatedBuffer_[contactIndexIt->second])
      {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
          "Ignored Contacts message with unknown or duplicated frame: %s",
          currentContactMessage.header.frame_id.c_str());
        return false;
      }

      const size_t currentIndex = contactIndexIt->second;
      newContactFlags[currentIndex] = std::pair<int, bool>(currentIndex,
        currentContactMessage.contact);
      contactUpdatedBuffer_[currentIndex] = true;

      const rclcpp::Time contactTime = currentContactMessage.header.stamp;
      if(newestContactTime < contactTime)
      {
        newestContactTime = contactTime;
      }
    }

    // Stamps can be unset (equal), but older contact states are not used
    if(contactsReceived_ && newestContactTime < lastContactFlagsTime_)
    {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Ignored Contacts message older than previous one");
      return false;
    }

    contactFlags_.swap(newContactFlags);
    lastContactFlagsTime_ = newestContactTime;
    lastContactsReceiveTime_ = this->get_clock()->now();
    contactsReceived_ = true;
    return true;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void LeggedStateEstimatorNode::processContacts(const contact_msgs::msg::Contacts::ConstSharedPtr contacts)
  {
    if(!estimatorRunning_) return;

    try
    {
      if(!readContacts(*contacts)) return;

      // Used by the following corrections with joint states
      leggedStateEstimator_->updateContacts(contactFlags_, toEstimatorTime(lastContactFlagsTime_));
    }
    catch(const std::exception& e)
    {
      // Exception would leave the executor and terminate the whole process
      RCLCPP_ERROR(this->get_logger(), "Estimator error: %s, estimator deactivation!", e.what());
      this->deactivate();
    }
  }


  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ocs2::scalar_t LeggedStateEstimatorNode::toEstimatorTime(const rclcpp::Time& time) const
  {
    // Relative time, absolute ROS time in double resolves only ~0.24 us
    return (time - estimatorStartTime_).seconds();
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  rclcpp::Time LeggedStateEstimatorNode::fromEstimatorTime(const ocs2::scalar_t time) const
  {
    // Rounded, so stamps of processed messages are restored exactly
    return estimatorStartTime_ + rclcpp::Duration(std::chrono::nanoseconds(std::llround(time * 1e9)));
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void LeggedStateEstimatorNode::processImu(const sensor_msgs::msg::Imu::ConstSharedPtr imu)
  {
    if(!estimatorRunning_)
    {
      // Before activation measurements are collected for the initial IMU biases
      if(readImu(*imu))
      {
        addImuCalibrationSample();
      }
      return;
    }

    try
    {
      const rclcpp::Time previousImuTime = lastImuTime_;
      
      if(!readImu(*imu)) return;

      // Gap between IMU samples, estimator holds the previous one during it
      const scalar_t dt = (lastImuTime_ - previousImuTime).seconds();

      if(dt > imuTimeout_.seconds())
      {
        RCLCPP_ERROR(this->get_logger(), 
          "Time between IMU messages is longer than imu_timeout, estimator deactivation!");
        this->deactivate();
        return;
      }

      // Estimator propagates to the IMU time and holds the new measurement
      leggedStateEstimator_->updateImu(angularVelocity_, linearAcceleration_, 
        toEstimatorTime(lastImuTime_));

      publishEstimates(fromEstimatorTime(leggedStateEstimator_->getEstimateTime()));
    }
    catch(const std::exception& e)
    {
      // Exception would leave the executor and terminate the whole process
      RCLCPP_ERROR(this->get_logger(), "Estimator error: %s, estimator deactivation!", e.what());
      this->deactivate();
    }
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void LeggedStateEstimatorNode::processJointStates(
    const sensor_msgs::msg::JointState::ConstSharedPtr jointStates)
  {
    if(!estimatorRunning_) return;

    try
    {
      const rclcpp::Time previousJointTime = lastJointStateTime_;

      if(!readJointStates(*jointStates)) return;

      // Gap between joint states messages
      const scalar_t dt = (lastJointStateTime_ - previousJointTime).seconds();

      if(dt > jointStateTimeout_.seconds())
      {
        RCLCPP_ERROR(this->get_logger(), 
          "Time between joint states messages is longer than joint_state_timeout, estimator deactivation!");
        this->deactivate();
        return;
      }

      // Last IMU measurement is held until joint states time, but not longer than imu_timeout
      if(lastJointStateTime_ - lastImuTime_ > imuTimeout_)
      {
        RCLCPP_ERROR(this->get_logger(), 
          "No IMU data for longer than imu_timeout before joint states, estimator deactivation!");
        this->deactivate();
        return;
      }

      // Estimator propagates to the joint states time with the held IMU measurement, then corrects
      // with contact states from the contact estimator or from the last contacts message
      leggedStateEstimator_->correct(jointPositions_, jointVelocities_, jointTorques_, 
        toEstimatorTime(lastJointStateTime_));

      estimateAvailable_ = true;

      publishEstimates(fromEstimatorTime(leggedStateEstimator_->getEstimateTime()));
    }
    catch(const std::exception& e)
    {
      // Exception would leave the executor and terminate the whole process
      RCLCPP_ERROR(this->get_logger(), "Estimator error: %s, estimator deactivation!", e.what());
      this->deactivate();
    }
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void LeggedStateEstimatorNode::checkSensorTimeouts()
  {
    if(!estimatorRunning_) return;

    const rclcpp::Time currentTime = this->get_clock()->now();

    if(!isImuFresh(currentTime))
    {
      RCLCPP_ERROR(this->get_logger(),
        "IMU data is older than imu_timeout, estimator deactivation!");
      this->deactivate();
      return;
    }

    if(!isJointStateFresh(currentTime))
    {
      RCLCPP_ERROR(this->get_logger(), 
        "Joint states are older than joint_state_timeout, estimator deactivation!");
      this->deactivate();
      return;
    }

    if(!leggedStateEstimator_->getSettings().use_contact_estimator && !isContactsFresh(currentTime))
    {
      RCLCPP_ERROR(this->get_logger(), 
        "No contacts message for longer than contacts_timeout, estimator deactivation!");
      this->deactivate();
      return;
    }
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void LeggedStateEstimatorNode::publishEstimates(const rclcpp::Time& stamp)
  {
    // Base pose and twist estimate. Pose is expressed in header frame (world),
    // twist in child frame (base), as in nav_msgs/Odometry definition
    const vector3_t& basePosition = leggedStateEstimator_->getBasePositionEstimate();
    const vector4_t& quaternionCoeffs = leggedStateEstimator_->getBaseQuaternionEstimate();
    const vector3_t& baseLinearVelocity = 
      leggedStateEstimator_->getBaseLinearVelocityEstimateLocal();
    const vector3_t& baseAngularVelocity = 
      leggedStateEstimator_->getBaseAngularVelocityEstimateLocal();

    // Preallocated message, frames are set in on_configure
    odometryMessage_.header.stamp = stamp;

    odometryMessage_.pose.pose.position = tf2::toMsg(basePosition);

    odometryMessage_.pose.pose.orientation.x = quaternionCoeffs(0);
    odometryMessage_.pose.pose.orientation.y = quaternionCoeffs(1);
    odometryMessage_.pose.pose.orientation.z = quaternionCoeffs(2);
    odometryMessage_.pose.pose.orientation.w = quaternionCoeffs(3);

    tf2::toMsg(baseLinearVelocity, odometryMessage_.twist.twist.linear);
    tf2::toMsg(baseAngularVelocity, odometryMessage_.twist.twist.angular);

    if(odometryEstimatePublisher_->is_activated())
    {
      odometryEstimatePublisher_->publish(odometryMessage_);
    }

    if(jointStatesEstimatePublisher_)
    {
      // Joint state estimates, written directly into preallocated message vectors
      const size_t jointNum = jointNames_.size();

      jointStatesMessage_.header.stamp = stamp;

      Eigen::Map<vector_t>(jointStatesMessage_.position.data(), jointNum) = jointPositions_;
      Eigen::Map<vector_t>(jointStatesMessage_.velocity.data(), jointNum) = 
        leggedStateEstimator_->getJointVelocityEstimate();
      Eigen::Map<vector_t>(jointStatesMessage_.effort.data(), jointNum) = 
        leggedStateEstimator_->getJointTorqueEstimate();

      if(jointStatesEstimatePublisher_->is_activated())
      {
        jointStatesEstimatePublisher_->publish(jointStatesMessage_);
      }
    }
  }
} // legged_state_estimator_ros2
