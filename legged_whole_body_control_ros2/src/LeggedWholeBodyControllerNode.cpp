#include <legged_whole_body_control_ros2/LeggedWholeBodyControllerNode.hpp>

#include <tf2_eigen/tf2_eigen.hpp>

#include <floating_base_model/FactoryFunctions.hpp>

#include <legged_whole_body_control/WeightedWbc.hpp>

namespace legged_whole_body_control_ros2
{
  using namespace ocs2;
  using namespace rclcpp;
  using namespace floating_base_model;
  using namespace legged_whole_body_control;
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  size_t numberOfClosedContacts(const FloatingBaseModelInfo &info,
    const contact_flags_t &contactFlags) 
  {
    size_t numEndEffectors = info.numThreeDofContacts + info.numSixDofContacts;

    return contactFlags.count() > numEndEffectors ? numEndEffectors : contactFlags.count();
  }
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  vector_t weightCompensatingInput(const FloatingBaseModelInfo &info, 
    const contact_flags_t &contactFlags)
  {
    const size_t numStanceLegs = numberOfClosedContacts(info, contactFlags);
    size_t numEndEffectors = info.numThreeDofContacts + info.numSixDofContacts;
    vector_t input = vector_t::Zero(info.inputDim);
    if(numStanceLegs > 0) 
    {
      const scalar_t totalWeight = info.robotMass * PLUS_GRAVITY_VALUE;
      const vector3_t forceInInertialFrame(0.0, 0.0, totalWeight / numStanceLegs);
      for (size_t i = 0; i < numEndEffectors; i++) 
      {
        if(contactFlags[i]) {
          access_helper_functions::getContactForces(input, i, info) = forceInInertialFrame;
        }
      } 
    }
    return input;
  }
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  LeggedWholeBodyControllerNode::LeggedWholeBodyControllerNode():
    LifecycleNode("legged_whole_body_controller", 
      NodeOptions().use_intra_process_comms(false))
  {
    // Config directory parameter
    this->declare_parameter("config_directory_path", "./");
    this->declare_parameter("urdf_path", "./");
    this->declare_parameter("type", "weighted");

    maxDurationBetweenMessages_ = rclcpp::Duration::from_seconds(1.0);

    lastJointStateTime_ = this->get_clock()->now();
    lastBaseTransformTime_ = this->get_clock()->now();
    lastBaseTwistTime_ = this->get_clock()->now();
    lastContactFlagsTime_ = this->get_clock()->now();
    lastDesiredObservationTime_ = this->get_clock()->now();

    RCLCPP_INFO(this->get_logger(), "Legged WBC Controller started in unconfigured state!");
  }
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    LeggedWholeBodyControllerNode::on_configure(const rclcpp_lifecycle::State& state)
  {
    /** 
     * Setup helper data structures
     */
    const std::string configDirectoryPath = this->get_parameter("config_directory_path").as_string();

    try
    {
      wbcSettings_ = loadWbcBaseSettings(configDirectoryPath + "/legged_wbc.info", 
        "wbc_base_settings", false);
    }
    catch(const std::exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "Error: Cannot find legged_wbc.info file!");
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    const std::string urdfFilePath = this->get_parameter("urdf_path").as_string();
    
    PinocchioInterface interface = createPinocchioInterfaceFromUrdfFile(urdfFilePath, 
      wbcSettings_.baseLinkName);

    modelInfo_ = createFloatingBaseModelInfo(interface, 
      wbcSettings_.endEffectorThreeDofNames, wbcSettings_.endEffectorSixDofNames);

    // Resize current measured observation
    currentMeasuredObservation_.state = vector_t::Zero(modelInfo_.stateDim);
    currentMeasuredObservation_.input = vector_t::Zero(modelInfo_.inputDim);

    // Resize current desired observation
    currentDesiredObservation_.state = vector_t::Zero(modelInfo_.stateDim);
    currentDesiredObservation_.input = vector_t::Zero(modelInfo_.inputDim);

    endEffectorNum_ = modelInfo_.numThreeDofContacts + modelInfo_.numSixDofContacts;

    for(size_t i = 0; i < wbcSettings_.endEffectorThreeDofNames.size(); ++i)
    {
      contactFrameNameIndexMap_[wbcSettings_.endEffectorThreeDofNames[i]] = i;
    }

    for(size_t i = wbcSettings_.endEffectorThreeDofNames.size();
      i < wbcSettings_.endEffectorThreeDofNames.size() + wbcSettings_.endEffectorSixDofNames.size(); ++i)
    {
      contactFrameNameIndexMap_[wbcSettings_.endEffectorSixDofNames[
        i - wbcSettings_.endEffectorThreeDofNames.size()]] = i;
    }

    const auto model = interface.getModel();

    jointNames_ = model.names;

    // Remove "universe" and "root_joint" joints from joint names
    jointNames_.erase(std::remove(jointNames_.begin(), jointNames_.end(), "universe"), jointNames_.end());
    jointNames_.erase(std::remove(jointNames_.begin(), jointNames_.end(), "root_joint"), jointNames_.end()); 

    for(size_t i = 0; i < jointNames_.size(); ++i)
    {
      jointNameIndexMap_[jointNames_[i]] = i;
    }

    /**
     * Create subsciber and publishers
     */

    // Callback group that will not be executed
    auto subscriptionOptions = rclcpp::SubscriptionOptions();
    rclcpp::CallbackGroup::SharedPtr cbGroupNotExecuted = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);
    subscriptionOptions.callback_group = cbGroupNotExecuted;
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos = qos.best_effort();

    // Desired mpc observation subscriber
    const std::string mpcObservationTopic = "/mpc_target_observation";
    desiredObservationSubscriber_ = this->create_subscription<ocs2_msgs::msg::MpcObservation>(
      mpcObservationTopic, qos, 
      [](const ocs2_msgs::msg::MpcObservation::ConstSharedPtr) {}, subscriptionOptions);

    const std::string terrianNormalsTopic = "/terrain_normals";
    terrainNormalsSubscriber_ = this->create_subscription<legged_locomotion_msgs::msg::TerrainNormalsStamped>(
      terrianNormalsTopic, qos, 
      [](const legged_locomotion_msgs::msg::TerrainNormalsStamped::ConstSharedPtr) {}, subscriptionOptions);
    
    // Joint states subscriber
    const std::string jointSatesTopic = "/joint_states";
    jointStateSubscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
      jointSatesTopic, qos, 
      [](const sensor_msgs::msg::JointState::ConstSharedPtr) {}, subscriptionOptions);
    
    // Base pose subscriber
    const std::string baseTransformTopic = "/base_transform";
    baseTransformSubscriber_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(
      baseTransformTopic, qos, 
      [](const geometry_msgs::msg::TransformStamped::ConstSharedPtr) {}, subscriptionOptions);

    // Base twist subscriber
    const std::string baseTwistTopic = "/base_twist";
    baseTwistSubscriber_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      baseTwistTopic, qos, 
      [](const geometry_msgs::msg::TwistStamped::ConstSharedPtr) {}, subscriptionOptions); 
    
    // Contact flags subscriber
    const std::string contactFlagsTopic = "/contacts";
    contactsSubscriber_ = this->create_subscription<contact_msgs::msg::Contacts>(
      contactFlagsTopic, qos, 
      [](const contact_msgs::msg::Contacts::ConstSharedPtr) {}, subscriptionOptions);

    // Base external wrench subsciber
    const std::string baseWrenchTopic = "/base_external_wrench";
    baseWrenchSubscriber_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
      baseWrenchTopic, QoS(1).reliable().keep_last(1), 
      std::bind(&LeggedWholeBodyControllerNode::updateExternalWrench, this, std::placeholders::_1));
    
    // Joint trajectory publisher
    const std::string jointTrajectoryTopic = "/joint_trajectory";
    jointTrajectoryPublisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      jointTrajectoryTopic, SystemDefaultsQoS());

    /**
     * Create timer for joint trajectory commands
     */

    // Get duration 
    wbcDurationSeconds_ = 1.0 / wbcSettings_.desiredFrequency;

    // Joint trajectory timer (not wall timer, as it uses system clock, not ROS one)
    jointTrajectoryTimer_ = rclcpp::create_timer(this, this->get_clock(), 
      rclcpp::Duration::from_seconds(wbcDurationSeconds_), 
      std::bind(&LeggedWholeBodyControllerNode::sendJointTrajectory, this));

    RCLCPP_INFO(this->get_logger(), "Legged WBC Controller configured successfully!");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    LeggedWholeBodyControllerNode::on_activate(const rclcpp_lifecycle::State& state)
  {
    try
    {
      setupWbc();
    }
    catch(const std::exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "WBC setup error: %s", e.what());
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    wbcBenchmarkTimer_.reset();

    jointTrajectoryPublisher_->on_activate();

    controllerRunning_ = true;

    RCLCPP_INFO(this->get_logger(), "Legged WBC Controller activated successfully!");
    RCLCPP_INFO(this->get_logger(), "WBC loop activated!");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    LeggedWholeBodyControllerNode::on_deactivate(const rclcpp_lifecycle::State& state)
  {
    controllerRunning_ = false;

    jointTrajectoryPublisher_->on_deactivate();

    std::string returnString;
    returnString += "########################################################################";
    returnString += "\n### WBC Benchmarking";
    returnString += "\n###   Maximum : " + std::to_string(wbcBenchmarkTimer_.getMaxIntervalInMilliseconds()) + "[ms].";
    returnString += "\n###   Average : " + std::to_string(wbcBenchmarkTimer_.getAverageInMilliseconds()) + "[ms].\n";

    RCLCPP_INFO(this->get_logger(), "%s", returnString.c_str());

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void LeggedWholeBodyControllerNode::updateExternalWrench(
    const geometry_msgs::msg::WrenchStamped::ConstSharedPtr externalWrench)
  {
    // Check if message has good base link in header
    if(externalWrench->header.frame_id != wbcSettings_.baseLinkName)
    {
      RCLCPP_ERROR(this->get_logger(), 
        "Base external wrench has wrong base frame ID!");
      return;
    }

    const rclcpp::Time currentTime = externalWrench->header.stamp;

    vector6_t baseWrench;
    baseWrench(0) = externalWrench->wrench.force.x;
    baseWrench(1) = externalWrench->wrench.force.y;
    baseWrench(2) = externalWrench->wrench.force.z;
    baseWrench(3) = externalWrench->wrench.torque.x;
    baseWrench(4) = externalWrench->wrench.torque.y;
    baseWrench(5) = externalWrench->wrench.torque.z;

    if(wbcPtr_ && controllerRunning_)
    {
      wbcPtr_->updateExternalWrench(currentTime.seconds(), baseWrench);
    }
  }
  
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void LeggedWholeBodyControllerNode::sendJointTrajectory()
  {
    if(wbcPtr_ && controllerRunning_)
    {
      wbcBenchmarkTimer_.startTimer();

      updateMeasuredObservation();
      updateDesiredObservation();
      updateContactFlags();
      updateTerrainNormals();

      wbcPtr_->updateDesired(currentDesiredObservation_.time, 
        currentDesiredObservation_.state, currentDesiredObservation_.input);

      wbcPtr_->updateMeasured(currentMeasuredObservation_.time, 
        currentMeasuredObservation_.state, currentMeasuredObservation_.input);

      wbcPtr_->updateContactFlags(lastContactFlagsTime_.seconds(), 
        contactFlags_);

      wbcPtr_->updateTerrainNormals(lastTerrainNormalsTime_.seconds(), terrainNormals_);

      wbcPtr_->calculate(currentMeasuredObservation_.time);

      trajectory_msgs::msg::JointTrajectory jointTrajectory;
      
      // Start trajectory now (time 0 seconds 0 nanoseconds)
      jointTrajectory.header.stamp.sec = 0;
      jointTrajectory.header.stamp.nanosec = 0;
      jointTrajectory.joint_names = jointNames_;
      
      jointTrajectory.points.resize(1);

      const auto accelerationVector = wbcPtr_->getJointAcceleration();

      const vector_t velocityVector = access_helper_functions::getJointVelocities(
        currentDesiredObservation_.input, modelInfo_) 
        + accelerationVector * wbcDurationSeconds_;

      const vector_t positionVector = access_helper_functions::getJointPositions(
        currentDesiredObservation_.state, modelInfo_) 
        + velocityVector * wbcDurationSeconds_;

      const vector_t effortVector = wbcPtr_->getJointTorque();

      const std::vector<scalar_t> positions(positionVector.data(), 
        positionVector.data() + positionVector.size());

      const std::vector<scalar_t> velocities(velocityVector.data(), 
        velocityVector.data() + velocityVector.size());

      const std::vector<scalar_t> efforts(effortVector.data(), 
          effortVector.data() + effortVector.size());
        
      jointTrajectory.points[0].time_from_start = rclcpp::Duration::from_seconds(0.0);
      jointTrajectory.points[0].positions = std::move(positions);
      jointTrajectory.points[0].velocities = std::move(velocities);
      jointTrajectory.points[0].effort = std::move(efforts);

      wbcBenchmarkTimer_.endTimer();

      // Publish 
      if(jointTrajectoryPublisher_->is_activated())
      {
        jointTrajectoryPublisher_->publish(jointTrajectory);
      }
    }
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void LeggedWholeBodyControllerNode::setupWbc()
  {
    const std::string urdfFilePath = this->get_parameter("urdf_path").as_string();
    
    PinocchioInterface interface = createPinocchioInterfaceFromUrdfFile(urdfFilePath, 
      wbcSettings_.baseLinkName);

    WeightedWbc::Weights weights;

    const std::string wbcType = this->get_parameter("type").as_string();

    if(wbcType == "weighted")
    {
      try
      {
        const std::string configDirectoryPath = this->get_parameter("config_directory_path").as_string();
        weights = loadWeightedWbcSettings(configDirectoryPath + "/legged_wbc.info", 
          "weighted_wbc_settings", false);
      }
      catch(const std::exception& e)
      {
        const std::string errorMessage = "Error: Cannot find legged_weighted_wbc.info file!";
        throw std::runtime_error(errorMessage);
      }
    }
    else
    {
      const std::string errorMessage = "Error: Only \"weighted\" type supported!";
      throw std::runtime_error(errorMessage);
    }

    wbcPtr_ = std::make_unique<WeightedWbc>(interface, modelInfo_, wbcSettings_, weights);

    updateMeasuredObservation();
    updateTerrainNormals();

    const size_t standingMode = ((0x01 << (endEffectorNum_)) - 1);
    const contact_flags_t standingFlags(standingMode);
    contactFlags_ = standingFlags;

    terrainNormals_.resize(endEffectorNum_, vector3_t{0.0, 0.0, 1.0});

    auto startInput = weightCompensatingInput(modelInfo_, contactFlags_);

    access_helper_functions::getJointVelocities(startInput, modelInfo_) = 
      access_helper_functions::getJointVelocities(currentMeasuredObservation_.input,
        modelInfo_);
    
    currentMeasuredObservation_.mode = standingMode;
    currentMeasuredObservation_.input = startInput;

    // Copy measured observation to desired at startup
    currentDesiredObservation_ = currentMeasuredObservation_;

    wbcPtr_->updateDesired(currentDesiredObservation_.time, 
        currentDesiredObservation_.state, currentDesiredObservation_.input);

    wbcPtr_->updateMeasured(currentMeasuredObservation_.time, 
      currentMeasuredObservation_.state, currentMeasuredObservation_.input);

    wbcPtr_->updateContactFlags(lastContactFlagsTime_.seconds(), 
      contactFlags_);

    wbcPtr_->updateTerrainNormals(lastTerrainNormalsTime_.seconds(), terrainNormals_);
  }
  
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void LeggedWholeBodyControllerNode::updateMeasuredObservation()
  {
    rclcpp::MessageInfo msgInfo;

    geometry_msgs::msg::TransformStamped baseTransform;
    if(baseTransformSubscriber_->take(baseTransform, msgInfo)) 
    {
      // Check if message has good base link in header
      if(baseTransform.header.frame_id != wbcSettings_.worldLinkName ||
         baseTransform.child_frame_id != wbcSettings_.baseLinkName)
      {
        RCLCPP_ERROR(this->get_logger(), 
          "Base transform has wrong base or world frame ID!");
        return;
      }

      if(this->get_clock()->now() - lastBaseTransformTime_ > maxDurationBetweenMessages_)
      {
        RCLCPP_WARN(this->get_logger(), 
          "Time between two TransformStamped messages was longer than maximum duration!");
      }

      lastBaseTransformTime_ = baseTransform.header.stamp;

      vector6_t currentBaseTransform;

      currentBaseTransform(0) = baseTransform.transform.translation.x;
      currentBaseTransform(1) = baseTransform.transform.translation.y;
      currentBaseTransform(2) = baseTransform.transform.translation.z;

      Eigen::Quaterniond quaterion;

      tf2::fromMsg(baseTransform.transform.rotation, quaterion);

      const matrix3_t rotationMatrix = quaterion.normalized().toRotationMatrix();

      const vector3_t eulerAngles = quaterion_euler_transforms::getEulerAnglesFromRotationMatrix(
        rotationMatrix);

      currentBaseTransform(3) = eulerAngles(0);
      currentBaseTransform(4) = eulerAngles(1);
      currentBaseTransform(5) = eulerAngles(2);

      access_helper_functions::getBasePose(
        currentMeasuredObservation_.state, modelInfo_) = currentBaseTransform;
    }

    geometry_msgs::msg::TwistStamped baseTwist;
    if(baseTwistSubscriber_->take(baseTwist, msgInfo)) 
    {
      // Check if message has good base link in header
      if(baseTwist.header.frame_id != wbcSettings_.baseLinkName)
      {
        RCLCPP_ERROR(this->get_logger(), 
          "Base twist has wrong base frame ID!");
        return;
      }

      if(this->get_clock()->now() - lastBaseTwistTime_ > maxDurationBetweenMessages_)
      {
        RCLCPP_WARN(this->get_logger(), 
          "Time between two TwistStamped messages was longer than maximum duration!");
      }

      lastBaseTwistTime_ = baseTwist.header.stamp;

      vector6_t currentBaseTwist;

      currentBaseTwist(0) = baseTwist.twist.linear.x;
      currentBaseTwist(1) = baseTwist.twist.linear.y;
      currentBaseTwist(2) = baseTwist.twist.linear.z;
      currentBaseTwist(3) = baseTwist.twist.angular.x;
      currentBaseTwist(4) = baseTwist.twist.angular.y;
      currentBaseTwist(5) = baseTwist.twist.angular.z;

      access_helper_functions::getBaseVelocity(
        currentMeasuredObservation_.state, modelInfo_) = currentBaseTwist;
    }

    sensor_msgs::msg::JointState jointStates;
    if(jointStateSubscriber_->take(jointStates, msgInfo)) 
    {
      // Check if message has same amount of data
      if(jointStates.name.size() != jointStates.position.size() || 
         jointStates.name.size() != jointStates.velocity.size() ||
         jointStates.name.size() != modelInfo_.actuatedDofNum)
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

      vector_t jointPositions = vector_t(modelInfo_.actuatedDofNum);
      vector_t jointVelocities = vector_t(modelInfo_.actuatedDofNum);

      for(size_t i = 0; i < jointStates.name.size(); ++i)
      {
        const size_t currentIndex = jointNameIndexMap_.at(jointStates.name[i]);
        jointPositions[currentIndex] = jointStates.position[i];
        jointVelocities[currentIndex] = jointStates.velocity[i];
      }

      access_helper_functions::getJointPositions(currentMeasuredObservation_.state, 
        modelInfo_) = jointPositions;
    }

    currentMeasuredObservation_.time = std::max(std::max(lastJointStateTime_.seconds(), 
      lastBaseTwistTime_.seconds()), lastBaseTransformTime_.seconds());
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void LeggedWholeBodyControllerNode::updateContactFlags()
  {
    rclcpp::MessageInfo msgInfo;

    contact_msgs::msg::Contacts contactFlags;
    if(contactsSubscriber_->take(contactFlags, msgInfo))
    {
      if(contactFlags.contacts.size() != endEffectorNum_)
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

      contactFlags_ = 0;

      for(size_t i = 0; i < contactFlags.contacts.size(); ++i)
      {
        const auto currentContactMessage = contactFlags.contacts[i];
        const size_t currentIndex = contactFrameNameIndexMap_[
          currentContactMessage.header.frame_id];
        contactFlags_[currentIndex] = currentContactMessage.contact;

        if(lastContactFlagsTime_ < currentContactMessage.header.stamp)
        {
          lastContactFlagsTime_ = currentContactMessage.header.stamp;
        }
      }
    }
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void LeggedWholeBodyControllerNode::updateTerrainNormals()
  {
    rclcpp::MessageInfo msgInfo;

    legged_locomotion_msgs::msg::TerrainNormalsStamped terrainNormalsMessage;
    if(terrainNormalsSubscriber_->take(terrainNormalsMessage, msgInfo)) 
    {
      if(terrainNormalsMessage.normals.size() != endEffectorNum_)
      {
        RCLCPP_ERROR(this->get_logger(), 
          "Ignored an invalid TerrainNormalsStamped message");
        return;
      }

      // Check if message has good base link in header
      if(this->get_clock()->now() - lastTerrainNormalsTime_ > maxDurationBetweenMessages_)
      {
        RCLCPP_WARN(this->get_logger(), 
          "Time between two TransformStamped messages was longer than maximum duration!");
      }

      lastBaseTransformTime_ = terrainNormalsMessage.header.stamp;

      const auto& normals = terrainNormalsMessage.normals;

      for(size_t i = 0; i < endEffectorNum_; ++i)
      {
        tf2::fromMsg(normals[i], terrainNormals_[i]);
      }
    }
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void LeggedWholeBodyControllerNode::updateDesiredObservation()
  {
    rclcpp::MessageInfo msgInfo;

    ocs2_msgs::msg::MpcObservation desiredObservation;
    if(desiredObservationSubscriber_->take(desiredObservation, msgInfo)) 
    {
      auto& desiredState = desiredObservation.state.value;
      auto& desiredInput = desiredObservation.input.value;

      if(desiredState.size() != modelInfo_.stateDim 
        || desiredInput.size() != modelInfo_.inputDim)
      {
        RCLCPP_ERROR(this->get_logger(), 
          "Ignored an invalid MpcObservation message");
        return;
      }

      // Check if message has good base link in header
      if(this->get_clock()->now() - lastDesiredObservationTime_ > maxDurationBetweenMessages_)
      {
        RCLCPP_WARN(this->get_logger(), 
          "Time between two desired observation messages was longer than maximum duration!");
      }

      lastDesiredObservationTime_ = rclcpp::Time(static_cast<int64_t>(
        desiredObservation.time * 1e9), RCL_ROS_TIME);

      currentDesiredObservation_.time = desiredObservation.time;
      currentDesiredObservation_.mode = desiredObservation.mode;
      currentDesiredObservation_.state = Eigen::Map<Eigen::VectorXf>(
        desiredState.data(), desiredState.size()).cast<scalar_t>();
      currentDesiredObservation_.input = Eigen::Map<Eigen::VectorXf>(
        desiredInput.data(), desiredInput.size()).cast<scalar_t>();
    }
  }
} // namespace legged_whole_body_control_ros2