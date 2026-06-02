#include <legged_locomotion_mpc_ros2/controller/LeggedMpcController.hpp>

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

namespace legged_locomotion_mpc_ros2
{
  using namespace ocs2;
  using namespace floating_base_model;
  using namespace legged_locomotion_mpc;
  using namespace terrain_model;
  using namespace rclcpp;
  using namespace rclcpp_lifecycle;
  using namespace grid_map;

  LeggedMpcController::LeggedMpcController(bool intraProcessComms):
    LifecycleNode("legged_controller", NodeOptions().use_intra_process_comms(intraProcessComms)),
    basePoseEstimation_(vector6_t::Zero()),
    baseTwistEstimation_(vector6_t::Zero()),
    jointPositionEstimation_(vector_t::Zero(10)),
    jointVelocityEstimation_(vector_t::Zero(10))
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

    maxDurationBetweenMessages_ = rclcpp::Duration::from_seconds(1.0);
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
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
    baseTwistSubscriber_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      baseTwistTopic, QoS(1).best_effort().keep_last(1), 
      std::bind(&LeggedMpcController::updateBaseTwist, this, std::placeholders::_1)); 
    
    // Contact flags subscriber
    const std::string contactFlagsTopic = this->get_parameter("contact_flags_topic").as_string();
    contactsSubscriber_ = this->create_subscription<contact_msgs::msg::Contacts>(
      contactFlagsTopic, QoS(1).reliable().keep_last(1), 
      std::bind(&LeggedMpcController::updateContactFlags, this, std::placeholders::_1));
    
    // Terrain subscriber
    const std::string terrainTopic = this->get_parameter("terrain_topic").as_string();
    terrainSubscriber_ = this->create_subscription<grid_map_msgs::msg::GridMap>(
      terrainTopic, QoS(1).reliable().keep_last(1), 
      std::bind(&LeggedMpcController::updateSegmentedTerrain, this, std::placeholders::_1));
    
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
    mrtDuration_ = 1.0 / mpcSettings.mrtDesiredFrequency_;

    // Joint trajectory timer (not wall timer, as it uses system clock, not ROS one)
    jointTrajectoryTimer_ = rclcpp::create_timer(this, this->get_clock(), 
      rclcpp::Duration::from_seconds(mrtDuration_), 
      std::bind(&LeggedMpcController::sendJointTrajectory, this));

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    LeggedMpcController::on_activate(const State& state)
  {
    try
    {
      setupMpc();
    }
    catch(const std::exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "[MPC setup] Error : %s", e.what());
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    runMpc();
    mpcTimer_.reset();
    mrtTimer_.reset();
    controllerRunning_ = true;
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    LeggedMpcController::on_deactivate(const State& state)
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
      
      access_helper_functions::getBaseVelocity(initialSystemState, 
        modelInfo_) = baseTwist;

      access_helper_functions::getJointPositions(initialSystemState, 
        modelInfo_) = jointPositions;
    }
    else
    {
      const std::string jointStatesTopic = this->get_parameter("joint_states_topic").as_string();
      const std::string basePoseTopic = this->get_parameter("base_pose_topic").as_string();
      const std::string baseTwistTopic = this->get_parameter("base_twist_topic").as_string();
      
      std::string errorMessage = "One of topics: " + jointStatesTopic + ", " + basePoseTopic + " or " + baseTwistTopic + " is not active!";
      RCLCPP_ERROR(this->get_logger(), "%s", errorMessage.c_str());
      throw std::runtime_error(errorMessage);
    }

    // Make plane terrain if terrain from subscriber is not available
    if(!terrainModelPtr_)
    {
      const vector3_t basePosition = access_helper_functions::getBasePosition(
        initialSystemState, modelInfo_);

      const vector3_t baseEulerAngles = access_helper_functions::getBaseOrientationZyx(
        initialSystemState, modelInfo_);

      const auto basePlannerSettings = planners::loadBasePlannerStaticSettings(
        modelFilePath, "base_planner_static_settings", false);

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
    referenceManagerPtr_ = &leggedInterfacePtr_->getLeggedReferenceManager();
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

    // Make rollout
    const auto& rollout = loopshapingInterfacePtr_->getRollout();
    rolloutPtr_.reset(rollout.clone());
    
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

    while(!mpcMrtPtr_->initialPolicyReceived() &&  
      (this->get_clock()->now() - startTime) < maxInitialPolicyDuration) 
    {
      mpcMrtPtr_->advanceMpc();
    }

    if(!mpcMrtPtr_->initialPolicyReceived())
    {
      RCLCPP_ERROR(this->get_logger(), 
        "Initial policy not recived, try again later!");
      throw std::runtime_error("Initial policy not recived, try again later!");
    }
    
    mpcMrtPtr_->initRollout(&leggedInterfacePtr_->getRollout());
  }

  SystemObservation LeggedMpcController::getCurrentObservation()
  {
    // Current loopshaping observation
    SystemObservation currentObservation;
    currentObservation.time = this->get_clock()->now().seconds();
    currentObservation.state = vector_t(modelInfo_.stateDim + modelInfo_.inputDim);
    currentObservation.input = vector_t::Zero(modelInfo_.inputDim);

    // Current system and filter state
    auto systemState = currentObservation.state.block(0, 0, modelInfo_.stateDim, 1);

    auto filterState = currentObservation.state.block(modelInfo_.stateDim, 0, 
      modelInfo_.inputDim, 1);

    if(!basePoseEstimation_.updateFromBuffer())
    {
      RCLCPP_WARN(this->get_logger(), 
        "Base pose not updated in current MPC iteration!");
    }

    if(!baseTwistEstimation_.updateFromBuffer())
    {
      RCLCPP_WARN(this->get_logger(), 
        "Base twist not updated in current MPC iteration!");
    }

    if(!jointPositionEstimation_.updateFromBuffer())
    {
      RCLCPP_WARN(this->get_logger(), 
        "Joint states not updated in current MPC iteration!");
    }

    access_helper_functions::getBasePose(
      systemState, modelInfo_) = basePoseEstimation_.get();

    access_helper_functions::getBaseVelocity(
      systemState, modelInfo_) = baseTwistEstimation_.get();

    access_helper_functions::getJointPositions(systemState, 
      modelInfo_) = jointPositionEstimation_.get();

    // Get filter state and input from last policy optimal trajectory
    const size_t queryIndex = utils::findIndexInTimeArray(
      mpcMrtPtr_->getPolicy().timeTrajectory_, currentObservation.time);

    filterState = mpcMrtPtr_->getPolicy().stateTrajectory_[queryIndex].block(
      modelInfo_.stateDim, 0, modelInfo_.inputDim, 1);

    currentObservation.input = mpcMrtPtr_->getPolicy().inputTrajectory_[queryIndex];

    return currentObservation;
  }

  void LeggedMpcController::runMpc()
  {
    mpcThread_ = std::thread([&]() 
    {
      while(controllerRunning_) 
      {
        try 
        {
          executeAndSleep([&]() 
          {
            mpcTimer_.startTimer();
            mpcMrtPtr_->advanceMpc();
            mpcTimer_.endTimer();
          }, leggedInterfacePtr_->mpcSettings().mpcDesiredFrequency_);
        } 
        catch(const std::exception& e) 
        {
          controllerRunning_ = false;
          RCLCPP_ERROR(this->get_logger(), "[MPC thread] Error : %s", e.what());
          this->trigger_transition(
            lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
        }
      }
    });
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

    if(referenceManagerPtr_ && controllerRunning_)
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

    lastBaseTwistTime_ = baseTwist->header.stamp;

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

    // TODO SPRAWDZ CZY TO NIE JEST TAK ZE KAZDY MA INNY TIMESTAMP
    lastContactFlagsTime_ = contactFlags->contacts[0].header.stamp;

    contact_flags_t contactFlagsData = 0;

    for(size_t i = 0; i < contactFlags->contacts.size(); ++i)
    {
      const auto currentContactMessage = contactFlags->contacts[i];
      const size_t currentIndex = contactFrameNameIndexMap_[
        currentContactMessage.header.frame_id];
      contactFlagsData[currentIndex] = currentContactMessage.contact;
    }

    if(referenceManagerPtr_ && controllerRunning_)
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
    parameters.swingRatio = gaitParameters->swing_ratio;
    parameters.phaseOffsets = gaitParameters->phase_offsets;

    if(referenceManagerPtr_ && controllerRunning_)
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

    if(referenceManagerPtr_ && controllerRunning_)
    {
      referenceManagerPtr_->updateSwingParameters(swingDynamicSettings);
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

    const rclcpp::Time currentTime = externalWrench->header.stamp;

    vector6_t baseWrench;
    baseWrench(0) = externalWrench->wrench.force.x;
    baseWrench(1) = externalWrench->wrench.force.y;
    baseWrench(2) = externalWrench->wrench.force.z;
    baseWrench(3) = externalWrench->wrench.torque.x;
    baseWrench(4) = externalWrench->wrench.torque.y;
    baseWrench(5) = externalWrench->wrench.torque.z;

    if(loopshapingInterfacePtr_ && controllerRunning_)
    {
      leggedInterfacePtr_->disturbanceModule().updateDistrubance(
        currentTime.seconds(), baseWrench);
    }
  }

  void LeggedMpcController::updateSegmentedTerrain(
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

  void LeggedMpcController::sendJointTrajectory()
  {
    if(mpcMrtPtr_ && controllerRunning_)
    {
      mrtTimer_.startTimer();
      const auto currentObservation = getCurrentObservation();

      mpcMrtPtr_->setCurrentObservation(currentObservation);

      mpcMrtPtr_->updatePolicy();

      const auto& currentState = currentObservation.state;
      
      const scalar_t currentTime = this->get_clock()->now().seconds();

      std::array<scalar_t, 2> times;
      std::array<vector_t, 2> optimizedStates;
      std::array<vector_t, 2> optimizedInputs;

      size_t plannedMode = 0;  
      vector_t loopshapingState;
      vector_t loopshapingInput;

      // Get current optimized system state and input
      times[0] = currentTime;
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
      rolloutPtr_->run(currentObservation.time, currentObservation.state, times[1],
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
          times[i] - currentTime);


        auto positionVector = access_helper_functions::getJointPositions(optimizedStates[i], 
          modelInfo_);

        const std::vector<scalar_t> positions(positionVector.data(), 
          positionVector.data() + positionVector.size());
        
        jointTrajectory->points[i].positions = std::move(positions);

        const auto velocityVector = loopshapingDefinitionPtr_->getSystemInput(
          optimizedStates[i], optimizedInputs[i]);

        const std::vector<scalar_t> velocities(velocityVector.data(), 
          velocityVector.data() + velocityVector.size());
        
        jointTrajectory->points[i].velocities = std::move(velocities);

        const auto effortVector = leggedInterfacePtr_->torqueApproximator().getValue(
          optimizedStates[i], optimizedInputs[i]);

        const std::vector<scalar_t> efforts(effortVector.data(), 
          effortVector.data() + effortVector.size());

        jointTrajectory->points[i].effort = std::move(efforts);
      }
      mrtTimer_.endTimer();

      // Publish 
      jointTrajectoryPublisher_->publish(std::move(jointTrajectory));
    }
  }
} // namespace legged_locomotion_mpc_ros2
