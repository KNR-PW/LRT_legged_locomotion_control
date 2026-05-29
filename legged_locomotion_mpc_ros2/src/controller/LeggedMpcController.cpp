#include <legged_locomotion_mpc_ros2/controller/LeggedMpcController.hpp>

#include <tf2_eigen/tf2_eigen.hpp>

#include <terrain_model/planar_model/PlanarFactoryFunctions.hpp>

#include <floating_base_model/AccessHelperFunctions.hpp>
#include <floating_base_model/QuaterionEulerTransforms.hpp>

namespace legged_locomotion_mpc_ros2
{
  using namespace floating_base_model;
  using namespace legged_locomotion_mpc;
  using namespace terrain_model;
  using namespace rclcpp;
  using namespace rclcpp_lifecycle;

  LeggedMpcController::LeggedMpcController(bool intraProcessComms):
    LifecycleNode("legged_controller", NodeOptions().use_intra_process_comms(intraProcessComms))
  {
    // Config directory parameter
    this->declare_parameter("config_directory_path", "./");
    this->declare_parameter("urdf_path", "./");

    // Topic name parameters
    this->declare_parameter("command_twist_topic", "~/commandTwist");
    this->declare_parameter("joint_states_topic", "~/joint_states");
    this->declare_parameter("base_pose_topic", "~/base_pose");
    this->declare_parameter("base_twist_topic", "~/base_twist");
    this->declare_parameter("contact_flags_topic", "~/contact_flags");
    this->declare_parameter("gait_parameters_topic", "~/gait_parameters");
    this->declare_parameter("swing_parameters_topic", "~/swing_parameters");
    this->declare_parameter("external_wrench_topic", "~/gait_parameters");
    this->declare_parameter("joint_trajectory_topic", "~/joint_forward_trajectory");

    configDirectoryPath_ = this->get_parameter("config_directory_path").as_string();


    maxDurationBetweenMessages_ = Time(1, 0, RCL_ROS_TIME);
  }

  node_interfaces::LifecycleNodeInterface::CallbackReturn
    LeggedMpcController::on_configure(const State& state)
  {

    /** 
     * Setup helper data structures
     */

    const std::string configDirectoryPath = this->get_parameter("config_directory_path").as_string();

    modelSettings_ = loadModelSettings(configDirectoryPath + "legged_model.info");

    for(size_t i = 0; i < modelSettings_.endEffectorThreeDofNames.size(); ++i)
    {
      contactFrameNameIndexMap_[modelSettings_.endEffectorThreeDofNames[i]] = i;
    }

    for(size_t i = modelSettings_.endEffectorThreeDofNames.size();
      i < modelSettings_.endEffectorThreeDofNames.size() + modelSettings_.endEffectorSixDofNames.size(); ++i)
    {
      contactFrameNameIndexMap_[modelSettings_.endEffectorThreeDofNames[
        i - modelSettings_.endEffectorThreeDofNames.size()]] = i;
    }

    const std::string urdfFilePath = this->get_parameter("urdf_path").as_string();
    PinocchioInterface interface = createPinocchioInterfaceFromUrdfFile(urdfFilePath, 
      modelSettings_.baseLinkName);
    modelInfo_ = createFloatingBaseModelInfo(interface, 
      modelSettings_.endEffectorThreeDofNames, modelSettings_.endEffectorSixDofNames);

    const auto model = interface.getModel();

    jointNames_ = model.names;

    // Remove "universe" and "root_joint" joints from joint names
    jointNames_.erase(std::remove(jointNames_.begin(), jointNames_.end(), "universe"), jointNames_.end());
    jointNames_.erase(std::remove(jointNames_.begin(), jointNames_.end(), "root_joint"), jointNames_.end()); 

    for(size_t i = 0; i < jointNames_.size(); ++i)
    {
      jointNameIndexMap_[jointNames_[i]] = i;
    }

    endEffectorNum_ = modelSettings_.endEffectorThreeDofNames.size() 
      + modelSettings_.endEffectorSixDofNames.size();

    stateOffset_ = 12;
    inputOffset_ = 3 * modelSettings_.endEffectorThreeDofNames.size() 
      + 6 * modelSettings_.endEffectorSixDofNames.size();

    /**
     * Create subsciber and publishers
     */

    // Command base twist subscriber
    const std::string twistTopic = this->get_parameter("command_twist_topic").as_string();
    commandTwistSubscriber_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      twistTopic, QoS(1).reliable().keep_last(1), 
      std::bind(&LeggedMpcController::updateCommand, this, std::placeholders::_1));
    
    // Joint states subscriber
    const std::string jointSatesTopic = this->get_parameter("joint_states_topic").as_string();
    jointStateSubscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
      jointSatesTopic, QoS(1).best_effort().keep_last(1), 
      std::bind(&LeggedMpcController::updateJointStates, this, std::placeholders::_1));
    
    // Base pose subscriber
    const std::string basePoseTopic = this->get_parameter("base_pose_topic").as_string();
    basePoseSubscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      basePoseTopic, QoS(1).best_effort().keep_last(1), 
      std::bind(&LeggedMpcController::updateBasePose, this, std::placeholders::_1));

    // Base twist (actual) subscriber
    const std::string baseTwistTopic = this->get_parameter("base_twist_topic").as_string();
    baseTwistSubscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      baseTwistTopic, QoS(1).best_effort().keep_last(1), 
      std::bind(&LeggedMpcController::updateBaseTwist, this, std::placeholders::_1)); 
    
    // Contact flags subscriber
    const std::string contactFlagsTopic = this->get_parameter("contact_flags_topic").as_string();
    contactsSubscriber_ = this->create_subscription<contact_msgs::msg::Contacts>(
      contactFlagsTopic, QoS(1).reliable().keep_last(1), 
      std::bind(&LeggedMpcController::updateContactFlags, this, std::placeholders::_1));
    
    // Gait parameters subscriber
    const std::string gaitParametersTopic = this->get_parameter("gait_parameters_topic").as_string();
    gaitParametersSubscriber_ = this->create_subscription<legged_locomotion_msgs::msg::GaitParameters>(
      gaitParametersTopic, QoS(1).reliable().keep_last(1), 
      std::bind(&LeggedMpcController::updateGaitParameters, this, std::placeholders::_1));

    // Swing paremeters subsciber
    const std::string swingParametersTopic = this->get_parameter("swing_parameters_topic").as_string();
    swingParametersSubscriber_ = this->create_subscription<legged_locomotion_msgs::msg::SwingParameters>(
      swingParametersTopic, QoS(1).reliable().keep_last(1), 
      std::bind(&LeggedMpcController::updateSwingParameters, this, std::placeholders::_1));

    // Base external wrench subsciber
    const std::string baseWrenchTopic = this->get_parameter("external_wrench_topic").as_string();
    baseWrenchSubscriber_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
      baseWrenchTopic, QoS(1).reliable().keep_last(1), 
      std::bind(&LeggedMpcController::updateExternalWrench, this, std::placeholders::_1));

    // Joint trajectory publisher
    const std::string jointTrajectoryTopic = this->get_parameter("joint_trajectory_topic").as_string();
    jointTrajectoryPublisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      jointTrajectoryTopic, SystemDefaultsQoS());


    /**
     * Create timer for joint trajectory commands
     */

    // Get duration 
    const auto mpcSettings = mpc::loadSettings(
      configDirectoryPath + "task.info", "mpc", false);
    mrtDuation_ = 1.0 / mpcSettings.mrtDesiredFrequency_;

    // Joint trajectory timer (not wall timer, as it uses system clock, not ROS one)
    jointTrajectoryTimer_ = rclcpp::create_timer(*this, this->get_clock(), 
      rclcpp::Duration::from_seconds(mrtDuration), 
      std::bind(&LeggedMpcController::sendJointTrajectory, this));
  }

  void LeggedMpcController::setupMpc()
  {
    const std::string configDirectoryPath = this->get_parameter("config_directory_path").as_string();
    const std::string taskFilePath = configDirectoryPath + "task.info";
    const std::string modelFilePath = configDirectoryPath + "legged_model.info";
    const std::string loopshapingFilePath = configDirectoryPath + "loopshaping.info";
    const std::string urdfFilePath = this->get_parameter("urdf_path").as_string();

    vector_t initialSystemState = vector_t(modelInfo_.stateDim);

    // Get initial system state
    if(basePoseEstimation_.updateFromBuffer() && 
       baseTwistEstimation_.updateFromBuffer() && 
       jointPositionEstimation_.updateFromBuffer())
    {
      const vector6_t basePose = basePoseEstimation_.get();
      const vector6_t baseTwist = baseTwistEstimation_.get();
      const vector_t jointPositions = jointPositionEstimation_.get();

      access_helper_functions::getBasePose(initialSystemState, 
        modelInfo_) = basePose;
      
      access_helper_functions::getBaseTwist(initialSystemState, 
        modelInfo_) = baseTwist;

      access_helper_functions::getJointPositions(initialSystemState, 
        modelInfo_) = jointPositions;
    }
    else
    {
      const std::string jointStatesTopic = this->get_parameter("joint_states_topic").as_string();
      const std::string basePoseTopic = this->get_parameter("base_pose_topic").as_string();
      const std::string baseTwistTopic = this->get_parameter("base_twist_topic").as_string();
      
      RCLCPP_ERROR(this->get_logger(), 
        "One of topics: %s, %s or %s is not active!", jointStatesTopic.c_str(), 
        basePoseTopic.c_str(), baseTwistTopic.c_str());
      throw std::runtime_error("One of topics: %s, %s or %s is not active!", 
        jointStatesTopic.c_str(), basePoseTopic.c_str(), baseTwistTopic.c_str());
    }

    // Make plane terrain if terrain from subscriber is not available
    if(!terrainModelPtr_)
    {
      const vector3_t basePosition = access_helper_functions::getBasePosition(
        initialSystemState, modelInfo_);

      const vector3_t baseEulerAngles = access_helper_functions::getBaseOrientationZyx(
        initialSystemState, modelInfo_);

      const auto basePlannerSettings = loadBasePlannerStaticSettings(modelFilePath
        "base_planner_static_settings", false);

      const TerrainPlane initalPlane = computeTerrainPlane(basePosition, baseEulerAngles, 
        basePlannerSettings.initialBaseHeight);

      terrainModelPtr_ = 
        std::make_unique<PlanarTerrainModel>(initalPlane, modelSettings_.worldLinkName);
    }

    const scalar_t initialTime = this->get_clock()->now().seconds();

    // Initialize loopshaping legged interface, 
    // legged interface and reference manager pointers
    loopshapingInterfacePtr_ = makeLeggedLoopshapingInterfacePointer(
      initialTime, initialSystemState, std::move(terrainModelPtr_), taskFilePath, 
      modelFilePath, urdfFilePath, loopshapingFilePath);

    leggedInterfacePtr_ = &loopshapingInterfacePtr_->getLeggedInterface();
    referenceManagerPtr_ = leggedInterfacePtr_->getLeggedReferenceManager();
    loopshapingDefinitionPtr_ = loopshapingInterfacePtr_->getLoopshapingDefinition().get();

    const auto& mpcSettings = leggedInterfacePtr_->mpcSettings();

    const auto& optimalProblem = loopshapingInterfacePtr_->getOptimalControlProblem();
    const auto& initializer = loopshapingInterfacePtr_->getInitializer();
    
    // Make MPC 
    if(modelSettings_.algorithm == "DDP")
    {
      const auto& ddpSettings = leggedInterfacePtr_->ddpSettings();
      auto& rollout = loopshapingInterfacePtr_->getRollout();
      mpcPtr_ = std::make_unique<GaussNewtonDDP_MPC>(mpcSettings, 
        ddpSettings, rollout, optimalProblem, initializer);
    }
    else if(modelSettings_.algorithm == "SQP")
    {
      const auto& sqpSettings = leggedInterfacePtr_->sqpSettings();
      mpcPtr_ = std::make_unique<SqpMpc>(mpcSettings, sqpSettings, 
        optimalProblem, initializer);
    }

    mpcPtr_->getSolverPtr()->setReferenceManager(
      loopshapingInterfacePtr_->getReferenceManagerPtr());
    mpcPtr_->getSolverPtr()->addSynchronizedModule(
      loopshapingInterfacePtr_->getSynchronizedModules());
    
    // Make MPC MRT interface
    mpcMrtPtr_ = std::make_unique<MPC_MRT_Interface>(*mpcPtr_);

    // Get observation of loopshaping model
    SystemObservation observation;
    observation.time = initialTime;
    observation.state = loopshapingInterfacePtr_->getInitialState();
    observation.input = vector_t::Zero(modelInfo_.inputDim);

    // Wait for the first policy
    mpcMrtPtr_->setCurrentObservation(observation);

    rclcpp::Duration maxInitialPolicyDuration = rclcpp::Duration::from_seconds(5);
    rclcpp::Time startTime = this->get_clock()->now();

    while(!mpcInterface.initialPolicyReceived() &&  
      (this->get_clock()->now() - startTime) < maxInitialPolicyDuration) 
    {
      mpcInterface.advanceMpc();
    }

    if(!mpcInterface.initialPolicyReceived())
    {
      RCLCPP_ERROR(this->get_logger(), 
        "Initial policy not recived, try again later!");
      throw std::runtime_error("Initial policy not recived, try again later!");
    }
    
    mpcMrtPtr_->initRollout(leggedInterfacePtr_->getRollout());

  }

  void LeggedMpcController::updateCommand(
    const geometry_msgs::msg::TwistStamped::ConstSharedPtr baseTwist)
  {
    // Check if message has good base link in header
    if(baseTwist->header.frame_id != modelSettings_.baseLinkName)
    {
      RCLCPP_ERROR(this->get_logger(), 
        "Base command has wrong base frame ID!");
      return;
    }

    planners::BaseTrajectoryPlanner::BaseReferenceCommand command;
    command.baseHeadingVelocity = baseTwist->twist.linear.x;
    command.baseLateralVelocity = baseTwist->twist.linear.y;
    command.baseVerticalVelocity = baseTwist->twist.linear.z;
    command.yawRate = baseTwist->twist.angular.z;

    if(referenceManagerPtr_ && mpcRunning_)
    {
      referenceManagerPtr_->updateCommand(command);
    }
  }

  void LeggedMpcController::updateJointStates(
    const sensor_msgs::msg::JointState::ConstSharedPtr jointStates)
  {
    // Check if message has same amount of data
    if(jointStates->name.size() != jointStates->position.size() || 
       jointStates->name.size() != jointStates->velocity.size() ||
       jointStates->name.size() != modelInfo_.actuatedDofNum)
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

    lastJointStateTime_ = jointStates->header.stamp;
    
    vector_t jointPositions = vector_t(modelInfo_.actuatedDofNum);
    vector_t jointVelocities = vector_t(modelInfo_.actuatedDofNum);
    
    for(size_t i = 0; i < jointStates->name.size(); ++i)
    {
      const size_t currentIndex = jointNameIndexMap_[jointStates->name[i]];
      jointPositions[currentIndex] = jointStates->position[i];
      jointVelocities[currentIndex] = jointStates->velocity[i];
    }

    jointPositionEstimation_.setBuffer(std::move(jointPositions));
    jointVelocityEstimation_.setBuffer(std::move(jointVelocities));
  }

  void LeggedMpcController::updateBasePose(
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr basePose)
  {
    // Check if message has good base link in header
    if(basePose->header.frame_id != modelSettings_.baseLinkName)
    {
      RCLCPP_ERROR(this->get_logger(), 
        "Base pose has wrong base frame ID!");
      return;
    }

    if(this->get_clock()->now() - lastBasePoseTime_ > maxDurationBetweenMessages_)
    {
      RCLCPP_WARN(this->get_logger(), 
        "Time between two PoseStamped messages was longer than maximum duration!");
    }

    lastBasePoseTime_ = basePose->header.stamp;

    vector6_t currentBasePose;

    currentBasePose(0) = basePose->pose.position.x;
    currentBasePose(1) = basePose->pose.position.y;
    currentBasePose(2) = basePose->pose.position.z;

    Eigen::Quaterniond quaterion;

    tf2::fromMsg(basePose->pose.orientation, quaterion);

    const matrix3_t rotationMatrix = quaterion.normalized().toRotationMatrix();

    const vector3_t eulerAngles = quaterion_euler_transforms::getEulerAnglesFromRotationMatrix(
      rotationMatrix);

    currentBasePose(3) = eulerAngles(1);
    currentBasePose(4) = eulerAngles(2);
    currentBasePose(5) = eulerAngles(3);

    basePoseEstimation_.setBuffer(std::move(currentBasePose));
  }

  void LeggedMpcController::updateBaseTwist(
    const geometry_msgs::msg::TwistStamped::ConstSharedPtr baseTwist)
  {
    // Check if message has good base link in header
    if(baseTwist->header.frame_id != modelSettings_.baseLinkName)
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

    lastBaseTwistTime_ = basePose->header.stamp;

    vector6_t currentBaseTwist;

    currentBaseTwist(0) = baseTwist->twist.linear.x;
    currentBaseTwist(1) = baseTwist->twist.linear.y;
    currentBaseTwist(2) = baseTwist->twist.linear.z;
    currentBaseTwist(3) = baseTwist->twist.angular.x;
    currentBaseTwist(4) = baseTwist->twist.angular.y;
    currentBaseTwist(5) = baseTwist->twist.angular.z;

    baseTwistEstimation_.setBuffer(std::move(currentBaseTwist));
  }

  void LeggedMpcController::updateContactFlags(
    const contact_msgs::msg::Contacts::ConstSharedPtr contactFlags)
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

    lastContactFlagsTime_.seconds = basePose->header.stamp.sec;
    lastContactFlagsTime_.nanoseconds = basePose->header.stamp.nanosec;

    contact_flags_t contactFlagsData = 0;

    for(size_t i = 0; i < contactFlags->contacts.size(); ++i)
    {
      const auto currentContactMessage = contactFlags->contacts[i];
      const size_t currentIndex = contactFrameNameIndexMap_[
        currentContactMessage.header.frame_id];
      contactFlagsData[currentIndex] = currentContactMessage.contact;
    }

    if(referenceManagerPtr_ && mpcRunning_)
    {
      referenceManagerPtr_->updateContactFlags(contactFlagsData);
    }
  }

  void LeggedMpcController::updateGaitParameters(
    const legged_locomotion_msgs::msg::GaitParameters::ConstSharedPtr gaitParameters)
  {
    if(gaitParameters->phase_offsets.size() != endEffectorNum_ - 1 || 
       gaitParameters->swing_ratio < 0 || gaitParameters->swing_ratio > 1 || 
       gaitParameters->stepping_frequency < 0)
    {
      RCLCPP_ERROR(this->get_logger(), 
        "Ignored an invalid GaitParameters message");
      return;
    }

    locomotion::GaitDynamicParameters parameters;
    parameters.steppingFrequency = gaitParameters->stepping_frequency;
    parameters.swingRatio = gaitParameters->swing_ratio
    parameters.phaseOffsets = gaitParameters->phase_offsets;

    if(referenceManagerPtr_ && mpcRunning_)
    {
      referenceManagerPtr_->updateGaitParemeters(parameters);
    }
  }

  void LeggedMpcController::updateSwingParameters(
    const legged_locomotion_msgs::msg::SwingParameters::ConstSharedPtr swingParameters)
  {
    if(swingParameters->phases.size() != endEffectorNum_ || 
       swingParameters->swing_heights.size() != endEffectorNum_ ||
       swingParameters->tangential_progresses.size() != endEffectorNum_ ||
       swingParameters->tangential_velocity_factors.size() != endEffectorNum_ ||
       swingParameters->inverted_pendulum_height < 0)
    {
      RCLCPP_ERROR(this->get_logger(), 
        "Ignored an invalid SwingParameters message");
      return;
    }

    using SwingDynamicSettings = locomotion::SwingTrajectoryPlanner::DynamicSettings;
    SwingDynamicSettings swingDynamicSettings;
    swingDynamicSettings.invertedPendulumHeight = swingParameters->inverted_pendulum_height;
    swingDynamicSettings.swingHeights = swingParameters->swing_heights;
    swingDynamicSettings.phases = swingParameters->phases;
    swingDynamicSettings.tangentialProgresses = swingParameters->tangential_progresses;
    swingDynamicSettings.tangentialVelocityFactors = swingParameters->tangential_velocity_factors;

    if(referenceManagerPtr_ && mpcRunning_)
    {
      referenceManagerPtr_->updateSwingParemeters(swingDynamicSettings);
    }
  }

  void LeggedMpcController::updateExternalWrench(
    const geometry_msgs::msg::WrenchStamped::ConstSharedPtr externalWrench)
  {
    // Check if message has good base link in header
    if(externalWrench->header.frame_id != modelSettings_.baseLinkName)
    {
      RCLCPP_ERROR(this->get_logger(), 
        "Base external wrench has wrong base frame ID!");
      return;
    }

    rclcpp::Time currentTime;

    currentTime.seconds = externalWrench->header.stamp.sec;
    currentTime.nanoseconds = externalWrench->header.stamp.nanosec;

    scalar_t currentTime = currentTime.seconds();

    vector6_t baseWrench;
    baseWrench(0) = externalWrench->wrench.force.x;
    baseWrench(1) = externalWrench->wrench.force.y;
    baseWrench(2) = externalWrench->wrench.force.z;
    baseWrench(3) = externalWrench->wrench.torque.x;
    baseWrench(4) = externalWrench->wrench.torque.y;
    baseWrench(5) = externalWrench->wrench.torque.z;

    if(loopshapingInterfacePtr_ && mpcRunning_)
    {
      leggedInterfacePtr_->disturbanceModule().updateDistrubance(
        currentTime, baseWrench);
    }
  }

  void LeggedMpcController::sendJointTrajectory()
  {
    if(mpcMrtPtr_ && mpcRunning_)
    {
      currentLoopshapingObservation_.updateFromBuffer();

      currentState = currentLoopshapingObservation_.get().state;
      
      const auto currentTime = this->get_clock()->now();

      std::array<scalar_t, 2> times;
      std::array<vector_t, 2> optimizedStates;
      std::array<vector_t, 2> optimizedInputs;

      size_t plannedMode = 0;  
      vector_t loopshapingState;
      vector_t loopshapingInput;

      // Get current optimized system state and input
      times[0] = currentTime.seconds();
      mpcMrtPtr_->evaluatePolicy(times[0],
        currentState, loopshapingState, loopshapingInput, plannedMode);

      optimizedStates[0] = loopshapingState.head(modelInfo_.stateDim);
      optimizedInputs[0] = loopshapingDefinitionPtr_->getSystemInput(loopshapingState, 
        loopshapingInput);

      // perform a rollout
      scalar_array_t timeTrajectory;
      size_array_t postEventIndicesStock;
      vector_array_t loopshapingStateTrajectory, loopshapingInputTrajectory;

      times[1] = times[0] + mrtDuration_;
      auto modeschedule = mpcMrtPtr_->getPolicy().modeSchedule_;
      rolloutPtr->run(observation.time, observation.state, times[1],
        mpcMrtPtr_->getPolicy().controllerPtr_.get(), modeschedule,
        timeTrajectory, postEventIndicesStock, loopshapingStateTrajectory,
        loopshapingInputTrajectory);
      
      // Get second optimized data
      times[1] = timeTrajectory.back();
      optimizedStates[1] = loopshapingStateTrajectory.back().head(modelInfo_.stateDim);
      optimizedInputs[1] = loopshapingDefinitionPtr_->getSystemInput(
        loopshapingStateTrajectory.back(), loopshapingInputTrajectory.back());

      std::unique_ptr<trajectory_msgs::msg::JointTrajectory> jointTrajectory;
      
      // Start trajectory now (time 0 seconds 0 nanoseconds)
      jointTrajectory->header.stamp.sec = 0;
      jointTrajectory->header.stamp.nanosec = 0;

      jointTrajectory->joint_names = jointNames_;
      jointTrajectory->points.reserve(2);
      
      for(size_t i = 0; i < times.size(); ++i)
      {
        jointTrajectory->points[i].time_from_start = rclcpp::Duration::from_seconds(
          times[i] - currentTime.seconds());
        
        jointTrajectory->points[i].positions = access_helper_functions::getJointPositions(
          optimizedStates[i], modelInfo_);
        
        jointTrajectory->points[i].velocities = access_helper_functions::getJointPositions(
          optimizedInputs[i], modelInfo_);

        jointTrajectory->points[i].effort = 
          leggedInterfacePtr_->torqueApproximator().getValue(
            optimizedStates[i], optimizedInputs[i]);
      }

      // Publish 
      jointTrajectoryPublisher_->publish(std::move(jointTrajectory));
    }
  }

} // namespace legged_locomotion_mpc_ros2
