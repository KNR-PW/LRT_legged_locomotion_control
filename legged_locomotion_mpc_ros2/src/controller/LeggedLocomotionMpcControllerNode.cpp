#include <legged_locomotion_mpc_ros2/controller/LeggedLocomotionMpcControllerNode.hpp>

#include <ocs2_core/thread_support/ExecuteAndSleep.h>

#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_sqp/SqpMpc.h>
#include <ocs2_ddp/GaussNewtonDDP_MPC.h>

#include <terrain_model/planar_model/PlanarFactoryFunctions.hpp>
#include <terrain_model/planar_model/PlanarTerrainModel.hpp>
#include <terrain_model/segmented_planes_model/SegmentedPlanesFactoryFunctions.hpp>
#include <terrain_model/segmented_planes_model/SegmentedPlanesTerrainModel.hpp>

#include <floating_base_model/FactoryFunctions.hpp>
#include <floating_base_model/AccessHelperFunctions.hpp>
#include <floating_base_model/QuaterionEulerTransforms.hpp>

#include <legged_locomotion_mpc/common/Utils.hpp>

#include <tf2_eigen/tf2_eigen.hpp>

#include <floating_base_model/PinocchioFloatingBaseDynamics.hpp>

namespace legged_locomotion_mpc_ros2
{
  using namespace ocs2;
  using namespace floating_base_model;
  using namespace legged_locomotion_mpc;
  using namespace terrain_model;
  using namespace rclcpp;
  using namespace rclcpp_lifecycle;
  using namespace grid_map;

  LeggedLocomotionMpcControllerNode::LeggedLocomotionMpcControllerNode():
    LifecycleNode("legged_locomotion_mpc_controller", NodeOptions().use_intra_process_comms(false))
  {
    // Config directory parameter
    this->declare_parameter("config_directory_path", "./");
    this->declare_parameter("urdf_path", "./");

    maxDurationBetweenMessages_ = rclcpp::Duration::from_seconds(1.0);

    lastJointStateTime_ = this->get_clock()->now();
    lastBaseTransformTime_ = this->get_clock()->now();
    lastBaseTwistTime_ = this->get_clock()->now();
    lastContactFlagsTime_ = this->get_clock()->now();

    RCLCPP_INFO(this->get_logger(), "Legged Locomotion MPC Controller started in unconfigured state!");
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    LeggedLocomotionMpcControllerNode::on_configure(const State& state)
  {
    /** 
     * Setup helper data structures
     */

    const std::string configDirectoryPath = this->get_parameter("config_directory_path").as_string();

    try
    {
      modelSettings_ = loadModelSettings(configDirectoryPath + "/legged_model.info", 
        "legged_model_settings", false);
    }
    catch(const std::exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "Error: Cannot find legged_model.info file!");
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    for(size_t i = 0; i < modelSettings_.endEffectorThreeDofNames.size(); ++i)
    {
      contactFrameNameIndexMap_[modelSettings_.endEffectorThreeDofNames[i]] = i;
    }

    for(size_t i = modelSettings_.endEffectorThreeDofNames.size();
      i < modelSettings_.endEffectorThreeDofNames.size() + modelSettings_.endEffectorSixDofNames.size(); ++i)
    {
      contactFrameNameIndexMap_[modelSettings_.endEffectorSixDofNames[
        i - modelSettings_.endEffectorThreeDofNames.size()]] = i;
    }

    const std::string urdfFilePath = this->get_parameter("urdf_path").as_string();
    PinocchioInterface interface = createPinocchioInterfaceFromUrdfFile(urdfFilePath, 
      modelSettings_.baseLinkName);
    modelInfo_ = createFloatingBaseModelInfo(interface, 
      modelSettings_.endEffectorThreeDofNames, modelSettings_.endEffectorSixDofNames);

    // Resize current observation
    currentObservation_.state = vector_t::Zero(modelInfo_.stateDim);
    currentObservation_.input = vector_t::Zero(modelInfo_.inputDim);

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

    /**
     * Prepare decompostion pipeline for segmented terrain model
     */

    // Get default decomposition pipeline config (for now at least)
    convex_plane_decomposition::PlaneDecompositionPipeline::Config config;

    decompositionPipelinePtr_ = std::make_unique<
      convex_plane_decomposition::PlaneDecompositionPipeline>(config);

    /**
     * Create subsciber and publishers
     */

    // Command base twist subscriber
    const std::string twistTopic = "/command_twist";
    commandTwistSubscriber_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      twistTopic, QoS(1).reliable().keep_last(1), 
      std::bind(&LeggedLocomotionMpcControllerNode::updateCommand, this, std::placeholders::_1));
    
    // Callback group that will not be executed
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
      [](const sensor_msgs::msg::JointState::ConstSharedPtr) {}, subscription_options);
    
    // Base pose subscriber
    const std::string baseTransformTopic = "/base_transform";
    baseTransformSubscriber_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(
      baseTransformTopic, qos, 
      [](const geometry_msgs::msg::TransformStamped::ConstSharedPtr) {}, subscription_options);

    // Base twist subscriber
    const std::string baseTwistTopic = "/base_twist";
    baseTwistSubscriber_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      baseTwistTopic, qos, 
      [](const geometry_msgs::msg::TwistStamped::ConstSharedPtr) {}, subscription_options); 
    
    // Contact flags subscriber
    const std::string contactFlagsTopic = "/contacts";
    contactsSubscriber_ = this->create_subscription<contact_msgs::msg::Contacts>(
      contactFlagsTopic, QoS(1).reliable().keep_last(1), 
      std::bind(&LeggedLocomotionMpcControllerNode::updateContactFlags, this, std::placeholders::_1));
    
    // Terrain subscriber
    const std::string terrainTopic = "/elevation";
    terrainSubscriber_ = this->create_subscription<grid_map_msgs::msg::GridMap>(
      terrainTopic, QoS(1).reliable().keep_last(1), 
      std::bind(&LeggedLocomotionMpcControllerNode::updateSegmentedTerrain, this, std::placeholders::_1));
    
    // Gait parameters subscriber
    const std::string gaitParametersTopic = "/gait_parameters";
    gaitParametersSubscriber_ = this->create_subscription<legged_locomotion_msgs::msg::GaitParameters>(
      gaitParametersTopic, QoS(1).reliable().keep_last(1), 
      std::bind(&LeggedLocomotionMpcControllerNode::updateGaitParameters, this, std::placeholders::_1));

    // Swing paremeters subsciber
    const std::string swingParametersTopic = "/swing_parameters";
    swingParametersSubscriber_ = this->create_subscription<legged_locomotion_msgs::msg::SwingParameters>(
      swingParametersTopic, QoS(1).reliable().keep_last(1), 
      std::bind(&LeggedLocomotionMpcControllerNode::updateSwingParameters, this, std::placeholders::_1));

    // Base external wrench subsciber
    const std::string baseWrenchTopic = "/base_external_wrench";
    baseWrenchSubscriber_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
      baseWrenchTopic, QoS(1).reliable().keep_last(1), 
      std::bind(&LeggedLocomotionMpcControllerNode::updateExternalWrench, this, std::placeholders::_1));

    // Joint trajectory publisher
    const std::string jointTrajectoryTopic = "/joint_trajectory";
    jointTrajectoryPublisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      jointTrajectoryTopic, SystemDefaultsQoS());

    referenceJointPublisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "reference_joint_states", SystemDefaultsQoS());

    optimizedJointPublisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "optimized_joint_states", SystemDefaultsQoS());

    RCLCPP_INFO(this->get_logger(), "Legged Locomotion MPC Controller configured successfully!");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    LeggedLocomotionMpcControllerNode::on_activate(const State& state)
  {
    try
    {
      setupMpc();
    }
    catch(const std::exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "MPC setup error: %s", e.what());
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    runMpc();
    mpcTimer_.reset();
    mrtTimer_.reset();
    controllerRunning_ = true;

    /**
     * Create timer for joint trajectory commands
     */

    // Get duration 
    const auto mpcSettings = leggedInterfacePtr_->mpcSettings();
    mrtDurationSeconds_ = 1.0 / mpcSettings.mrtDesiredFrequency_;

    mpcDuration_ = rclcpp::Duration::from_seconds(1.0 / mpcSettings.mpcDesiredFrequency_);

    // Joint trajectory timer (not wall timer, as it uses system clock, not ROS one)
    jointTrajectoryTimer_ = rclcpp::create_timer(this, this->get_clock(), 
      rclcpp::Duration::from_seconds(mrtDurationSeconds_), 
      std::bind(&LeggedLocomotionMpcControllerNode::sendJointTrajectory, this));

    
    RCLCPP_INFO(this->get_logger(), "Legged Locomotion MPC Controller activated successfully!");
    RCLCPP_INFO(this->get_logger(), "MPC/MRT loop activated!");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    LeggedLocomotionMpcControllerNode::on_deactivate(const State& state)
  {
    controllerRunning_ = false;
    if (mpcThread_.joinable()) mpcThread_.join();
    std::string returnString;

    returnString += "########################################################################";
    returnString += "\n### MPC Benchmarking";
    returnString += "\n###   Maximum : " + std::to_string(mpcTimer_.getMaxIntervalInMilliseconds()) + "[ms].";
    returnString += "\n###   Average : " + std::to_string(mpcTimer_.getAverageInMilliseconds()) + "[ms].\n";
    returnString += "########################################################################";
    returnString += "\n### WRT Benchmarking";
    returnString += "\n###   Maximum : " + std::to_string(mrtTimer_.getMaxIntervalInMilliseconds()) + "[ms].";
    returnString += "\n###   Average : " + std::to_string(mpcTimer_.getAverageInMilliseconds()) + "[ms].\n";

    RCLCPP_INFO(this->get_logger(), "%s", returnString.c_str());

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  void LeggedLocomotionMpcControllerNode::setupMpc()
  {
    const std::string configDirectoryPath = this->get_parameter("config_directory_path").as_string();
    const std::string taskFilePath = configDirectoryPath + "/task.info";
    const std::string modelFilePath = configDirectoryPath + "/legged_model.info";
    const std::string urdfFilePath = this->get_parameter("urdf_path").as_string();

    // Get initial system state
    updateCurrentObservation();

    vector_t initialSystemState = currentObservation_.state;

    // Make plane terrain if terrain from subscriber is not available
    if(!terrainModelPtr_)
    {      
      RCLCPP_WARN(this->get_logger(), 
        "Did not get any segmented plane terrain, using plane terrain with position [0,0,0] and rotation [0,0,0]!");
      
        const TerrainPlane initalPlane(vector3_t::Zero(), matrix3_t::Identity());

      terrainModelPtr_ = 
        std::make_unique<PlanarTerrainModel>(initalPlane, modelSettings_.worldLinkName);
    }

    const scalar_t initialTime = this->get_clock()->now().seconds();

    // Initialize loopshaping legged interface, 
    // legged interface and reference manager pointers
    leggedInterfacePtr_ = std::make_unique<LeggedInterface>(
      initialTime, initialSystemState, std::move(terrainModelPtr_), taskFilePath, 
      modelFilePath, urdfFilePath);

    referenceManagerPtr_ = &leggedInterfacePtr_->getLeggedReferenceManager();
  
    const auto& mpcSettings = leggedInterfacePtr_->mpcSettings();

    const auto& optimalProblem = leggedInterfacePtr_->getOptimalControlProblem();
    const auto& initializer = leggedInterfacePtr_->getInitializer();

    // Make MPC 
    if(modelSettings_.algorithm == "DDP")
    {
      const auto& ddpSettings = leggedInterfacePtr_->ddpSettings();
      auto& rollout = leggedInterfacePtr_->getRollout();
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
      leggedInterfacePtr_->getReferenceManagerPtr());
    mpcPtr_->getSolverPtr()->setSynchronizedModules(
      leggedInterfacePtr_->getSynchronizedModulePtrs());

    // Make rollout
    auto& rollout = leggedInterfacePtr_->getRollout();
    rolloutPtr_.reset(rollout.clone());
    
    // Make MPC MRT interface
    mpcMrtPtr_ = std::make_unique<MPC_MRT_Interface>(*mpcPtr_);

    const auto& modelInfo = leggedInterfacePtr_->floatingBaseModelInfo();
    
    const size_t endEffectorNum = modelInfo.numThreeDofContacts 
      + modelInfo.numSixDofContacts;
    const size_t standingMode = ((0x01 << (endEffectorNum)) - 1);
    const contact_flags_t standingFlags(standingMode);

    auto& weightCompensator = leggedInterfacePtr_->weightCompensator();

    const auto& initalSystemState = leggedInterfacePtr_->getInitialState();

    const vector_t initialInput = weightCompensator.getInput(initalSystemState, 
      standingFlags);

    // Get observation of the model
    currentObservation_.time = this->get_clock()->now().seconds();
    currentObservation_.state = leggedInterfacePtr_->getInitialState();
    currentObservation_.input = initialInput;

    // Wait for the first policy
    mpcMrtPtr_->setCurrentObservation(currentObservation_);

    rclcpp::Duration maxInitialPolicyDuration = rclcpp::Duration::from_seconds(10.0);
    rclcpp::Time startTime = this->get_clock()->now();

    while(!mpcMrtPtr_->initialPolicyReceived() && rclcpp::ok() && 
      (this->get_clock()->now() - startTime) < maxInitialPolicyDuration) 
    {
      mpcMrtPtr_->advanceMpc();
    }

    if(!mpcMrtPtr_->initialPolicyReceived())
    {
      const std::string errorMessage = "Initial policy not recived, try again later!";
      throw std::runtime_error(errorMessage);
    }
    
    mpcMrtPtr_->initRollout(&leggedInterfacePtr_->getRollout());
  }

  void LeggedLocomotionMpcControllerNode::updateCurrentObservation()
  {
    rclcpp::MessageInfo msgInfo;

    geometry_msgs::msg::TransformStamped baseTransform;
    if(baseTransformSubscriber_->take(baseTransform, msgInfo)) 
    {
      // Check if message has good base link in header
      if(baseTransform.header.frame_id != modelSettings_.worldLinkName ||
         baseTransform.child_frame_id != modelSettings_.baseLinkName)
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
        currentObservation_.state, modelInfo_) = currentBaseTransform;
    }

    geometry_msgs::msg::TwistStamped baseTwist;
    if(baseTwistSubscriber_->take(baseTwist, msgInfo)) 
    {
      // Check if message has good base link in header
      if(baseTwist.header.frame_id != modelSettings_.baseLinkName)
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
        currentObservation_.state, modelInfo_) = currentBaseTwist;
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

      access_helper_functions::getJointPositions(currentObservation_.state, 
        modelInfo_) = jointPositions;
    }

    currentObservation_.time = std::max(std::max(lastJointStateTime_.seconds(), 
      lastBaseTwistTime_.seconds()), lastBaseTransformTime_.seconds());
  }

  void LeggedLocomotionMpcControllerNode::runMpc()
  {
    mpcThread_ = std::thread([&]() 
    {
      while(controllerRunning_) 
      {
        try 
        {
          const auto startTime = this->get_clock()->now();
          mpcTimer_.startTimer();
          mpcMrtPtr_->advanceMpc();
          mpcTimer_.endTimer();
          const auto endTime = this->get_clock()->now();
          const auto durationLeft = mpcDuration_ - (endTime - startTime);
          this->get_clock()->sleep_for(durationLeft);
          RCLCPP_INFO(this->get_logger(), "MPC iteration took : %f", (endTime - startTime).seconds());
          RCLCPP_INFO(this->get_logger(), "MPC duration left : %f", durationLeft.seconds());
        } 
        catch(const std::exception& e) 
        {
          controllerRunning_ = false;
          RCLCPP_ERROR(this->get_logger(), "MPC thread error : %s", e.what());
          RCLCPP_ERROR(this->get_logger(), "MPC loop deactivation!");
          this->trigger_transition(
            lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
        }
      }
    });
  }

  void LeggedLocomotionMpcControllerNode::updateCommand(
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

    if(referenceManagerPtr_ && controllerRunning_)
    {
      referenceManagerPtr_->updateCommand(command);
    }
  }

  void LeggedLocomotionMpcControllerNode::updateContactFlags(
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

    contact_flags_t contactFlagsData = 0;

    for(size_t i = 0; i < contactFlags->contacts.size(); ++i)
    {
      const auto currentContactMessage = contactFlags->contacts[i];
      const size_t currentIndex = contactFrameNameIndexMap_[
        currentContactMessage.header.frame_id];
      contactFlagsData[currentIndex] = currentContactMessage.contact;

      if(lastContactFlagsTime_ < currentContactMessage.header.stamp)
      {
        lastContactFlagsTime_ = currentContactMessage.header.stamp;
      }
    }

    if(referenceManagerPtr_ && controllerRunning_)
    {
      referenceManagerPtr_->updateContactFlags(contactFlagsData);
    }
  }

  void LeggedLocomotionMpcControllerNode::updateGaitParameters(
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
    parameters.swingRatio = gaitParameters->swing_ratio;
    parameters.phaseOffsets = gaitParameters->phase_offsets;

    if(referenceManagerPtr_ && controllerRunning_)
    {
      referenceManagerPtr_->updateGaitParemeters(parameters);
    }
  }

  void LeggedLocomotionMpcControllerNode::updateSwingParameters(
    const legged_locomotion_msgs::msg::SwingParameters::ConstSharedPtr swingParameters)
  {
    if(swingParameters->phases.size() != endEffectorNum_ || 
       swingParameters->swing_heights.size() != endEffectorNum_ ||
       swingParameters->tangential_progresses.size() != endEffectorNum_ ||
       swingParameters->tangential_velocity_factors.size() != endEffectorNum_)
    {
      RCLCPP_ERROR(this->get_logger(), 
        "Ignored an invalid SwingParameters message");
      return;
    }

    using SwingDynamicSettings = locomotion::SwingTrajectoryPlanner::DynamicSettings;
    SwingDynamicSettings swingDynamicSettings;
    swingDynamicSettings.swingHeights = swingParameters->swing_heights;
    swingDynamicSettings.phases = swingParameters->phases;
    swingDynamicSettings.tangentialProgresses = swingParameters->tangential_progresses;
    swingDynamicSettings.tangentialVelocityFactors = swingParameters->tangential_velocity_factors;

    if(referenceManagerPtr_ && controllerRunning_)
    {
      referenceManagerPtr_->updateSwingParameters(swingDynamicSettings);
    }
  }

  void LeggedLocomotionMpcControllerNode::updateExternalWrench(
    const geometry_msgs::msg::WrenchStamped::ConstSharedPtr externalWrench)
  {
    // Check if message has good base link in header
    if(externalWrench->header.frame_id != modelSettings_.baseLinkName)
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

    if(leggedInterfacePtr_ && controllerRunning_)
    {
      leggedInterfacePtr_->disturbanceModule().updateDistrubance(
        currentTime.seconds(), baseWrench);
    }
  }

  void LeggedLocomotionMpcControllerNode::updateSegmentedTerrain(
    grid_map_msgs::msg::GridMap::UniquePtr gridMap)
  {
    if(!decompositionPipelinePtr_) return;
 
    // Make gridMap object
    grid_map::GridMap elevationMap;

    if(!GridMapRosConverter::fromMessage(*gridMap, elevationMap, 
      {terrain_model::elevationLayerName}, true, false))
    {
      RCLCPP_ERROR(this->get_logger(), 
        "Error during GridMapRosConverter::fromMessage()!");
      return;
    }

    decompositionPipelinePtr_->update(std::move(elevationMap), 
      terrain_model::elevationLayerName);

    auto planarTerrain = decompositionPipelinePtr_->movePlanarTerrain();

    terrainModelPtr_ = std::make_unique<terrain_model::SegmentedPlanesTerrainModel>(
      std::move(planarTerrain), modelSettings_.worldLinkName); 

    if(referenceManagerPtr_ && controllerRunning_)
    {
      referenceManagerPtr_->updateTerrainModel(std::move(terrainModelPtr_));
    }
  }

  void LeggedLocomotionMpcControllerNode::sendJointTrajectory()
  {
    
    if(mpcMrtPtr_ && controllerRunning_)
    {
      RCLCPP_INFO(this->get_logger(), "MRT iteration starting at : %f", 
        this->get_clock()->now().seconds());

      RCLCPP_INFO(this->get_logger(), "Observation at time : %f", 
        currentObservation_.time);
      
      mrtTimer_.startTimer();

      updateCurrentObservation();

      mpcMrtPtr_->setCurrentObservation(currentObservation_);

      mpcMrtPtr_->updatePolicy();

      const auto& currentState = currentObservation_.state;

      size_t plannedMode = 0;  
      vector_t optimizedState;
      vector_t optimizedInput;

      // Get current optimal state and input
      mpcMrtPtr_->evaluatePolicy(currentObservation_.time,
        currentState, optimizedState, optimizedInput, plannedMode);

      currentObservation_.input = optimizedInput;

      // Get optimal state and input at time: currentObservation_.time + mrtDuration_
      scalar_t nextTime = currentObservation_.time + mrtDurationSeconds_;
      const auto& timeTrajectory = mpcMrtPtr_->getPolicy().timeTrajectory_;
      const auto& stateTrajectory = mpcMrtPtr_->getPolicy().stateTrajectory_;
      const auto& inputTrajectory = mpcMrtPtr_->getPolicy().inputTrajectory_;

      const auto nextState = LinearInterpolation::interpolate(nextTime, 
        timeTrajectory, stateTrajectory);

      const auto nextInput = LinearInterpolation::interpolate(nextTime, 
        timeTrajectory, inputTrajectory);

      trajectory_msgs::msg::JointTrajectory jointTrajectory;
      
      // Start trajectory now (time 0 seconds 0 nanoseconds)
      jointTrajectory.header.stamp.sec = 0;
      jointTrajectory.header.stamp.nanosec = 0;

      // TODO ZMIEN
      jointTrajectory.joint_names = jointNames_;
      for(size_t i = 0; i < jointNames_.size(); ++i)
      {
        jointTrajectory.joint_names[i] = "joint_controller/" + jointTrajectory.joint_names[i];
      }

      jointTrajectory.points.resize(2);

      const vector_t firstPositionVector = access_helper_functions::getJointPositions(
        optimizedState, modelInfo_);

      const vector_t firstVelocityVector = access_helper_functions::getJointVelocities(
        optimizedInput, modelInfo_);

      const vector_t firstEffortVector = leggedInterfacePtr_->torqueApproximator().getValue(
          optimizedState, optimizedInput);

      const std::vector<scalar_t> firstPositions(firstPositionVector.data(), 
        firstPositionVector.data() + firstPositionVector.size());

      const std::vector<scalar_t> firstVelocities(firstVelocityVector.data(), 
        firstVelocityVector.data() + firstVelocityVector.size());

      const std::vector<scalar_t> firstEfforts(firstEffortVector.data(), 
          firstEffortVector.data() + firstEffortVector.size());
        
      jointTrajectory.points[0].time_from_start = rclcpp::Duration::from_seconds(0.0); //rclcpp::Duration::from_seconds(mrtDurationSeconds_);
      // jointTrajectory.points[0].positions = std::move(positions);
      // jointTrajectory.points[0].velocities = std::move(velocities);
      // jointTrajectory.points[0].effort = std::move(efforts);
      jointTrajectory.points[0].positions = firstPositions;
      jointTrajectory.points[0].velocities = firstVelocities;
      jointTrajectory.points[0].effort = firstEfforts;

      const vector_t secondPositionVector = access_helper_functions::getJointPositions(
        nextState, modelInfo_);

      const vector_t secondVelocityVector = access_helper_functions::getJointVelocities(
        nextInput, modelInfo_);

      const vector_t secondEffortVector = leggedInterfacePtr_->torqueApproximator().getValue(
          nextState, nextInput);

      const std::vector<scalar_t> secondPositions(secondPositionVector.data(), 
        secondPositionVector.data() + secondPositionVector.size());

      const std::vector<scalar_t> secondVelocities(secondVelocityVector.data(), 
        secondVelocityVector.data() + secondVelocityVector.size());

      const std::vector<scalar_t> secondEfforts(secondEffortVector.data(), 
          secondEffortVector.data() + secondEffortVector.size());
        
      jointTrajectory.points[1].time_from_start = rclcpp::Duration::from_seconds(
        mrtDurationSeconds_);
      // jointTrajectory.points[0].positions = std::move(positions);
      // jointTrajectory.points[0].velocities = std::move(velocities);
      // jointTrajectory.points[0].effort = std::move(efforts);
      jointTrajectory.points[1].positions = secondPositions;
      jointTrajectory.points[1].velocities = secondVelocities;
      jointTrajectory.points[1].effort = secondEfforts;

      mrtTimer_.endTimer();

      const auto& trajectory = referenceManagerPtr_->getTargetTrajectories();

      const auto referencePositionVector = access_helper_functions::getJointPositions(
        trajectory.stateTrajectory.front(), modelInfo_);

      const auto referenceVelocityVector = access_helper_functions::getJointVelocities(
        trajectory.inputTrajectory.front(), modelInfo_);

       const std::vector<scalar_t> referencePositions(referencePositionVector.data(), 
        referencePositionVector.data() + referencePositionVector.size());

      const std::vector<scalar_t> refefrenceVelocities(referenceVelocityVector.data(), 
        referenceVelocityVector.data() + referenceVelocityVector.size());

      sensor_msgs::msg::JointState referenceJointStates;
      referenceJointStates.name = jointNames_;
      referenceJointStates.position = std::move(referencePositions);
      referenceJointStates.velocity = std::move(refefrenceVelocities);

      sensor_msgs::msg::JointState optimizedJointStates;
      optimizedJointStates.name = jointNames_;
      optimizedJointStates.position = firstPositions;
      optimizedJointStates.velocity = firstVelocities;
      optimizedJointStates.effort = firstEfforts;

      // Publish 
      jointTrajectoryPublisher_->publish(jointTrajectory);
      referenceJointPublisher_->publish(std::move(referenceJointStates));
      optimizedJointPublisher_->publish(std::move(optimizedJointStates));
    }
  }
} // namespace legged_locomotion_mpc_ros2
