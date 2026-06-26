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
    LifecycleNode("legged_mpc_controller", NodeOptions().use_intra_process_comms(intraProcessComms)),
    baseTransformEstimation_(vector6_t::Zero()),
    baseTwistEstimation_(vector6_t::Zero()),
    jointPositionEstimation_(vector_t()),
    jointVelocityEstimation_(vector_t())
  {
    // Config directory parameter
    this->declare_parameter("config_directory_path", "./");
    this->declare_parameter("urdf_path", "./");

    maxDurationBetweenMessages_ = rclcpp::Duration::from_seconds(1.0);

    lastJointStateTime_ = this->get_clock()->now();
    lastBaseTransformTime_ = this->get_clock()->now();
    lastBaseTwistTime_ = this->get_clock()->now();
    lastContactFlagsTime_ = this->get_clock()->now();

    RCLCPP_INFO(this->get_logger(), "Legged MPC Controller started in unconfigured state!");
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    LeggedMpcController::on_configure(const State& state)
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
    const std::string twistTopic = "/command_twist";
    commandTwistSubscriber_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      twistTopic, QoS(1).reliable().keep_last(1), 
      std::bind(&LeggedMpcController::updateCommand, this, std::placeholders::_1));
    
    // Joint states subscriber
    const std::string jointSatesTopic = "/joint_states";
    jointStateSubscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
      jointSatesTopic, QoS(1).best_effort().keep_last(1), 
      std::bind(&LeggedMpcController::updateJointStates, this, std::placeholders::_1));
    
    // Base pose subscriber
    const std::string baseTransformTopic = "/base_transform";
    baseTransformSubscriber_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(
      baseTransformTopic, QoS(1).best_effort().keep_last(1), 
      std::bind(&LeggedMpcController::updateBaseTransform, this, std::placeholders::_1));

    // Base twist (actual) subscriber
    const std::string baseTwistTopic = "/base_twist";
    baseTwistSubscriber_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      baseTwistTopic, QoS(1).best_effort().keep_last(1), 
      std::bind(&LeggedMpcController::updateBaseTwist, this, std::placeholders::_1)); 
    
    // Contact flags subscriber
    const std::string contactFlagsTopic = "/contact_flags";
    contactsSubscriber_ = this->create_subscription<contact_msgs::msg::Contacts>(
      contactFlagsTopic, QoS(1).reliable().keep_last(1), 
      std::bind(&LeggedMpcController::updateContactFlags, this, std::placeholders::_1));
    
    // Terrain subscriber
    const std::string terrainTopic = "/elevation";
    terrainSubscriber_ = this->create_subscription<grid_map_msgs::msg::GridMap>(
      terrainTopic, QoS(1).reliable().keep_last(1), 
      std::bind(&LeggedMpcController::updateSegmentedTerrain, this, std::placeholders::_1));
    
    // Gait parameters subscriber
    const std::string gaitParametersTopic = "/gait_parameters";
    gaitParametersSubscriber_ = this->create_subscription<legged_locomotion_msgs::msg::GaitParameters>(
      gaitParametersTopic, QoS(1).reliable().keep_last(1), 
      std::bind(&LeggedMpcController::updateGaitParameters, this, std::placeholders::_1));

    // Swing paremeters subsciber
    const std::string swingParametersTopic = "/swing_parameters";
    swingParametersSubscriber_ = this->create_subscription<legged_locomotion_msgs::msg::SwingParameters>(
      swingParametersTopic, QoS(1).reliable().keep_last(1), 
      std::bind(&LeggedMpcController::updateSwingParameters, this, std::placeholders::_1));

    // Base external wrench subsciber
    const std::string baseWrenchTopic = "/base_external_wrench";
    baseWrenchSubscriber_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
      baseWrenchTopic, QoS(1).reliable().keep_last(1), 
      std::bind(&LeggedMpcController::updateExternalWrench, this, std::placeholders::_1));

    // Joint trajectory publisher
    const std::string jointTrajectoryTopic = "/joint_trajectory";
    jointTrajectoryPublisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      jointTrajectoryTopic, SystemDefaultsQoS());

    /**
     * Create timer for joint trajectory commands
     */

    // Get duration 
    const auto mpcSettings = mpc::loadSettings(
      configDirectoryPath + "/task.info", "mpc", false);
    mrtDurationSeconds_ = 1.0 / mpcSettings.mrtDesiredFrequency_;

    mpcDuration_ = rclcpp::Duration::from_seconds(1.0 / mpcSettings.mpcDesiredFrequency_);

    // Joint trajectory timer (not wall timer, as it uses system clock, not ROS one)
    jointTrajectoryTimer_ = rclcpp::create_timer(this, this->get_clock(), 
      rclcpp::Duration::from_seconds(mrtDurationSeconds_), 
      std::bind(&LeggedMpcController::sendJointTrajectory, this));

    RCLCPP_INFO(this->get_logger(), "Legged MPC Controller configured successfully!");
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
      RCLCPP_ERROR(this->get_logger(), "MPC setup error: %s", e.what());
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    runMpc();
    mpcTimer_.reset();
    mrtTimer_.reset();
    controllerRunning_ = true;
    RCLCPP_INFO(this->get_logger(), "Legged MPC Controller activated successfully!");
    RCLCPP_INFO(this->get_logger(), "MPC/MRT loop activated!");

    // jointStateSubscriber_.reset();
    // baseTransformSubscriber_.reset();
    // baseTwistSubscriber_.reset();
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
    const std::string taskFilePath = configDirectoryPath + "/task.info";
    const std::string modelFilePath = configDirectoryPath + "/legged_model.info";
    // const std::string loopshapingFilePath = configDirectoryPath + "/loopshaping.info";
    const std::string urdfFilePath = this->get_parameter("urdf_path").as_string();

    vector_t initialSystemState = vector_t(modelInfo_.stateDim);

    // Get initial system state
    if(baseTransformEstimation_.updateFromBuffer() && 
       baseTwistEstimation_.updateFromBuffer() && 
       jointPositionEstimation_.updateFromBuffer())
    {
      const vector6_t baseTransform = baseTransformEstimation_.get();
      const vector6_t baseTwist = baseTwistEstimation_.get();
      const vector_t jointPositions = jointPositionEstimation_.get();

      // std::stringstream ss;
      // ss << "Init state: " << "\n";
      // ss << "Base position: " << baseTransform.transpose() << "\n";
      // ss << "Base velocity: " << baseTwist.transpose() << "\n";
      // ss << "Joint positions: " << jointPositions.transpose() << "\n";

      // RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());

      access_helper_functions::getBasePose(initialSystemState, 
        modelInfo_) = baseTransform;
      
      access_helper_functions::getBaseVelocity(initialSystemState, 
        modelInfo_) = baseTwist;

      access_helper_functions::getJointPositions(initialSystemState, 
        modelInfo_) = jointPositions;
    }
    else
    {
      const std::string jointStatesTopic = std::string(jointStateSubscriber_->get_topic_name());
      const std::string baseTransformTopic = std::string(baseTransformSubscriber_->get_topic_name());
      const std::string baseTwistTopic = std::string(baseTwistSubscriber_->get_topic_name());
      
      std::string errorMessage = "One of topics: " + jointStatesTopic + ", " + baseTransformTopic + " or " + baseTwistTopic + " is not active!";
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
    const auto& rollout = leggedInterfacePtr_->getRollout();
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

    // Get observation of loopshaping model
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

    // Make one iteration to get current observation after initialization
    mpcMrtPtr_->updatePolicy();
    currentObservation_ = getCurrentObservation();

    {
      const vector_t newState = currentObservation_.state;

      const vector6_t baseTransform = access_helper_functions::getBasePose(newState, 
        modelInfo_);
      const vector6_t baseTwist = access_helper_functions::getBaseVelocity(newState, 
        modelInfo_);
      const vector_t jointPositions = access_helper_functions::getJointPositions(newState, 
        modelInfo_);

      // std::stringstream ss;
      // ss << "New init state: " << "\n";
      // ss << "Base position: " << baseTransform.transpose() << "\n";
      // ss << "Base velocity: " << baseTwist.transpose() << "\n";
      // ss << "Joint positions: " << jointPositions.transpose() << "\n";
      // RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
    }

    mpcMrtPtr_->setCurrentObservation(currentObservation_);
    mpcMrtPtr_->advanceMpc();
  }

  SystemObservation LeggedMpcController::getCurrentObservation()
  {
    // Current loopshaping observation
    SystemObservation currentObservation;
    currentObservation.time = this->get_clock()->now().seconds();
    currentObservation.state = vector_t(modelInfo_.stateDim);

    if(!baseTransformEstimation_.updateFromBuffer())
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
      currentObservation.state, modelInfo_) = baseTransformEstimation_.get();

    access_helper_functions::getBaseVelocity(
      currentObservation.state, modelInfo_) = baseTwistEstimation_.get();

    access_helper_functions::getJointPositions(currentObservation.state, 
      modelInfo_) = jointPositionEstimation_.get();

    currentObservation.input = vector_t::Zero(modelInfo_.inputDim);

    // currentObservation.input = mpcMrtPtr_->getPolicy().inputTrajectory_[queryIndex];

    // const auto systemInput = loopshapingDefinitionPtr_->getSystemInput(currentObservation.state, currentObservation.input);

    // std::stringstream ss;

    // ss << "Current observation: " << "\n";
    // ss << "State: " << currentObservation.state.head(modelInfo_.stateDim).transpose() << "\n";
    // // ss << "Filter state: " << currentObservation.state.tail(modelInfo_.stateDim).transpose() << std::endl;
    // RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
    // rclcpp::shutdown();
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
    // RCLCPP_INFO(this->get_logger(), "AAAAAAAA!");
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

  void LeggedMpcController::updateBaseTransform(
    const geometry_msgs::msg::TransformStamped::ConstSharedPtr baseTransform)
  {
    // Check if message has good base link in header
    if(baseTransform->header.frame_id != modelSettings_.worldLinkName ||
       baseTransform->child_frame_id != modelSettings_.baseLinkName)
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

    lastBaseTransformTime_ = baseTransform->header.stamp;

    vector6_t currentBaseTransform;

    currentBaseTransform(0) = baseTransform->transform.translation.x;
    currentBaseTransform(1) = baseTransform->transform.translation.y;
    currentBaseTransform(2) = baseTransform->transform.translation.z;

    Eigen::Quaterniond quaterion;

    tf2::fromMsg(baseTransform->transform.rotation, quaterion);

    const matrix3_t rotationMatrix = quaterion.normalized().toRotationMatrix();

    const vector3_t eulerAngles = quaterion_euler_transforms::getEulerAnglesFromRotationMatrix(
      rotationMatrix);

    currentBaseTransform(3) = eulerAngles(0);
    currentBaseTransform(4) = eulerAngles(1);
    currentBaseTransform(5) = eulerAngles(2);

    baseTransformEstimation_.setBuffer(std::move(currentBaseTransform));
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

    if(leggedInterfacePtr_ && controllerRunning_)
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
      RCLCPP_INFO(this->get_logger(), "MRT iteration starting at : %f", this->get_clock()->now().seconds());
      mrtTimer_.startTimer();

      currentObservation_ = getCurrentObservation();

      {
      const vector_t newState = currentObservation_.state;

      const vector6_t baseTransform = access_helper_functions::getBasePose(newState, 
        modelInfo_);
      const vector6_t baseTwist = access_helper_functions::getBaseVelocity(newState, 
        modelInfo_);
      const vector_t jointPositions = access_helper_functions::getJointPositions(newState, 
        modelInfo_);

      // std::stringstream ss;
      // ss << "New state: " << "\n";
      // ss << "Base position: " << baseTransform.transpose() << "\n";
      // ss << "Base velocity: " << baseTwist.transpose() << "\n";
      // ss << "Joint positions: " << jointPositions.transpose() << "\n";
      // RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
    } 

      mpcMrtPtr_->setCurrentObservation(currentObservation_);

      mpcMrtPtr_->updatePolicy();

      const auto& currentState = currentObservation_.state;

      const auto trajectory = referenceManagerPtr_->getTargetTrajectories();

      const vector3_t basePosition = trajectory.stateTrajectory.front().block<3, 1>(6, 0);
      const vector3_t currentBase = currentObservation_.state.block<3, 1>(6, 0);

      // std::stringstream ss;
      // ss << "Base optimal position: " << basePosition.transpose() << "\n";
      // ss << "Base current position: " << currentBase.transpose() << "\n";

      // RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
      
      // const scalar_t currentTime = this->get_clock()->now().seconds();

      size_t plannedMode = 0;  
      vector_t optimizedState;
      vector_t optimizedInput;

      mpcMrtPtr_->evaluatePolicy(currentObservation_.time,
        currentState, optimizedState, optimizedInput, plannedMode);

      currentObservation_.input = optimizedInput;

      // RCLCPP_INFO(this->get_logger(), "Planned mode: %d", plannedMode);


      // std::stringstream ss2;
      // ss2 << "Optimized state: " << optimizedState.transpose() << "\n";
      // ss2 << "Optimized input: " << optimizedInput.transpose() << "\n";
      
      // RCLCPP_INFO(this->get_logger(), "%s", ss2.str().c_str());


      // std::array<scalar_t, 2> times;
      // std::array<vector_t, 2> optimizedStates;
      // std::array<vector_t, 2> optimizedInputs;

      // size_t plannedMode = 0;  
      // vector_t loopshapingState;
      // vector_t loopshapingInput;

      // // Get current optimized system state and input
      // times[0] = currentTime;
      // mpcMrtPtr_->evaluatePolicy(times[0],
      //   currentState, loopshapingState, loopshapingInput, plannedMode);

      // optimizedStates[0] = loopshapingState.head(modelInfo_.stateDim);
      // optimizedInputs[0] = access_helper_functions::getJointVelocities(loopshapingDefinitionPtr_->getSystemInput(loopshapingState, 
      //   loopshapingInput), modelInfo_);

      // // perform a rollout
      // scalar_array_t timeTrajectory;
      // size_array_t postEventIndicesStock;
      // vector_array_t loopshapingStateTrajectory, loopshapingInputTrajectory;

      // times[1] = times[0] + mrtDurationSeconds_;
      // auto modeschedule = mpcMrtPtr_->getPolicy().modeSchedule_;
      // rolloutPtr_->run(currentObservation.time, currentObservation.state, times[1],
      //   mpcMrtPtr_->getPolicy().controllerPtr_.get(), modeschedule,
      //   timeTrajectory, postEventIndicesStock, loopshapingStateTrajectory,
      //   loopshapingInputTrajectory);
      
      // // Get second optimized data
      // times[1] = timeTrajectory.back();
      // optimizedStates[1] = loopshapingStateTrajectory.back().head(modelInfo_.stateDim);
      // optimizedInputs[1] = access_helper_functions::getJointVelocities(loopshapingDefinitionPtr_->getSystemInput(
      //   loopshapingStateTrajectory.back(), loopshapingInputTrajectory.back()), modelInfo_);

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

      // jointTrajectory.points.resize(2);
      
      // for(size_t i = 0; i < times.size(); ++i)
      // {
      //   jointTrajectory.points[i].time_from_start = rclcpp::Duration::from_seconds(
      //     times[i] - currentTime);


      //   auto positionVector = access_helper_functions::getJointPositions(optimizedStates[i], 
      //     modelInfo_);

      //   const std::vector<scalar_t> positions(positionVector.data(), 
      //     positionVector.data() + positionVector.size());
        
      //   jointTrajectory.points[i].positions = std::move(positions);

      //   const auto velocityVector = loopshapingDefinitionPtr_->getSystemInput(
      //     optimizedStates[i], optimizedInputs[i]);

      //   const std::vector<scalar_t> velocities(velocityVector.data(), 
      //     velocityVector.data() + velocityVector.size());
        
      //   // jointTrajectory.points[i].velocities = std::move(velocities);
      //   jointTrajectory.points[i].velocities = std::vector(modelInfo_.actuatedDofNum, 0.0);

      //   const auto effortVector = leggedInterfacePtr_->torqueApproximator().getValue(
      //     optimizedStates[i], optimizedInputs[i]);

      //   const std::vector<scalar_t> efforts(effortVector.data(), 
      //     effortVector.data() + effortVector.size());

      //   // jointTrajectory.points[i].effort = std::move(efforts);
      //   jointTrajectory.points[i].effort = std::vector(modelInfo_.actuatedDofNum, 0.0);
      // }

      jointTrajectory.points.resize(1);

      const auto positionVector = access_helper_functions::getJointPositions(
        trajectory.stateTrajectory.front(), modelInfo_);

      const auto velocityVector = access_helper_functions::getJointVelocities(
        optimizedInput, modelInfo_);

      const auto effortVector = leggedInterfacePtr_->torqueApproximator().getValue(
          optimizedState, optimizedInput);

      const std::vector<scalar_t> positions(positionVector.data(), 
        positionVector.data() + positionVector.size());

      const std::vector<scalar_t> velocities(velocityVector.data(), 
        velocityVector.data() + velocityVector.size());

      const std::vector<scalar_t> efforts(effortVector.data(), 
          effortVector.data() + effortVector.size());
        
      jointTrajectory.points[0].time_from_start = rclcpp::Duration::from_seconds(0.1);
      jointTrajectory.points[0].positions = std::move(positions);
      // jointTrajectory.points[0].velocities = std::move(velocities);
      // jointTrajectory.points[0].effort = std::move(efforts);
      jointTrajectory.points[0].velocities = std::vector(modelInfo_.actuatedDofNum, 0.0);
      jointTrajectory.points[0].effort = std::vector(modelInfo_.actuatedDofNum, 0.0);

      mrtTimer_.endTimer();

      // Publish 
      jointTrajectoryPublisher_->publish(std::move(jointTrajectory));
    }
  }
} // namespace legged_locomotion_mpc_ros2
