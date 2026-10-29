#include <legged_locomotion_mpc_ros2/controller/LeggedLocomotionMpcControllerNode.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>

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
#include <legged_locomotion_mpc/locomotion/GaitParameters.hpp>

#include <tf2_eigen/tf2_eigen.hpp>

#include <lifecycle_msgs/msg/state.hpp>

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

    // Maximum age of robot state messages [s], controller is deactivated after that
    this->declare_parameter("state_timeout", 0.1);

    // Maximum duration between base velocity commands [s], robot stops after that
    this->declare_parameter("velocity_timeout", 0.1);

    // Maximum age of MPC policy [s], controller is deactivated after that (stalled MPC)
    this->declare_parameter("policy_timeout", 0.5);

    // Same clock type as node clock, times from different clocks can not be compared
    const rclcpp::Time zeroTime(0, 0, this->get_clock()->get_clock_type());
    lastJointStateTime_ = zeroTime;
    lastOdometryTime_ = zeroTime;
    lastCommandTime_ = zeroTime;
    mpcStartTime_ = zeroTime;

    RCLCPP_INFO(this->get_logger(), "Legged Locomotion MPC Controller started in unconfigured state!");
  }

  LeggedLocomotionMpcControllerNode::~LeggedLocomotionMpcControllerNode()
  {
    // Joinable thread can not be destroyed (std::terminate)
    stopMpcThread();
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    LeggedLocomotionMpcControllerNode::on_configure(const State& state)
  {
    /** 
     * Setup helper data structures
     */

    const std::string configDirectoryPath = this->get_parameter("config_directory_path").as_string();
    const std::string modelFilePath = configDirectoryPath + "/legged_model.info";

    try
    {
      modelSettings_ = loadModelSettings(modelFilePath, "legged_model_settings", false);

      // Gait limits for validation of gait parameters messages
      const auto gaitStaticParameters = locomotion::loadGaitStaticParameters(modelFilePath,
        modelSettings_, "gait_static_settings", false);
      minimumSteppingFrequency_ = gaitStaticParameters.minimumSteppingFrequency;
    }
    catch(const std::exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "Error while loading legged_model.info file: %s", e.what());
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    const scalar_t stateTimeout = this->get_parameter("state_timeout").as_double();
    const scalar_t velocityTimeout = this->get_parameter("velocity_timeout").as_double();
    const scalar_t policyTimeout = this->get_parameter("policy_timeout").as_double();

    if(!(stateTimeout > 0.0) || !(velocityTimeout > 0.0) || !(policyTimeout > 0.0))
    {
      RCLCPP_ERROR(this->get_logger(),
        "Parameters state_timeout, velocity_timeout and policy_timeout have to be positive!");
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    stateTimeout_ = rclcpp::Duration::from_seconds(stateTimeout);
    velocityTimeout_ = rclcpp::Duration::from_seconds(velocityTimeout);
    policyTimeout_ = policyTimeout;

    contactFrameNameIndexMap_.clear();

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

    // Buffers for joint states and contacts in MRT loop
    jointPositionsBuffer_ = vector_t::Zero(modelInfo_.actuatedDofNum);
    jointUpdatedBuffer_.assign(modelInfo_.actuatedDofNum, false);
    contactUpdatedBuffer_.assign(modelSettings_.endEffectorThreeDofNames.size()
      + modelSettings_.endEffectorSixDofNames.size(), false);

    const auto model = interface.getModel();

    jointNames_ = model.names;

    // Remove "universe" and "root_joint" joints from joint names
    jointNames_.erase(std::remove(jointNames_.begin(), jointNames_.end(), "universe"), jointNames_.end());
    jointNames_.erase(std::remove(jointNames_.begin(), jointNames_.end(), "root_joint"), jointNames_.end()); 

    jointNameIndexMap_.clear();

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

    {
      // Terrain callback might be running in parallel (reconfiguration)
      std::lock_guard<std::mutex> lock(terrainMutex_);
      decompositionPipelinePtr_ = std::make_shared<
        convex_plane_decomposition::PlaneDecompositionPipeline>(config);
    }

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
    
    // Base pose and twist subscriber
    const std::string odometryTopic = "/odom";
    odometrySubscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odometryTopic, qos,
      [](const nav_msgs::msg::Odometry::ConstSharedPtr) {}, subscription_options);
    
    // Contact flags subscriber
    const std::string contactFlagsTopic = "/contacts";
    contactsSubscriber_ = this->create_subscription<contact_msgs::msg::Contacts>(
      contactFlagsTopic, qos, 
      [](const contact_msgs::msg::Contacts::ConstSharedPtr) {}, subscription_options);
    
    // Terrain subscriber, own callback group as plane decomposition is slow
    terrainCallbackGroup_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    auto terrainSubscriptionOptions = rclcpp::SubscriptionOptions();
    terrainSubscriptionOptions.callback_group = terrainCallbackGroup_;

    const std::string terrainTopic = "/elevation";
    terrainSubscriber_ = this->create_subscription<grid_map_msgs::msg::GridMap>(
      terrainTopic, QoS(1).reliable().keep_last(1), 
      std::bind(&LeggedLocomotionMpcControllerNode::updateSegmentedTerrain, this, std::placeholders::_1),
      terrainSubscriptionOptions);
    
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
    
    // Joint trajectory publisher, only newest trajectory matters. Volatile, so subscriber
    // started later does not get old trajectory (system default is transient local for Fast DDS)
    const std::string jointTrajectoryTopic = "/joint_trajectory";
    jointTrajectoryPublisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      jointTrajectoryTopic, QoS(1).reliable().durability_volatile());

    /**
     * Create timer for joint trajectory commands
     */

    // Get duration 
    const std::string taskFilePath = configDirectoryPath + "/task.info";
    const auto mpcSettings = mpc::loadSettings(taskFilePath, "mpc", false);
    mrtDurationSeconds_ = 1.0 / mpcSettings.mrtDesiredFrequency_;

    mpcDuration_ = rclcpp::Duration::from_seconds(1.0 / mpcSettings.mpcDesiredFrequency_);

    // Preallocate joint trajectory message, MRT loop only writes point values
    jointTrajectory_ = trajectory_msgs::msg::JointTrajectory();

    // Start trajectory now (time 0 seconds 0 nanoseconds)
    jointTrajectory_.header.stamp.sec = 0;
    jointTrajectory_.header.stamp.nanosec = 0;
    jointTrajectory_.joint_names = jointNames_;

    jointTrajectory_.points.resize(2);
    for(auto& point: jointTrajectory_.points)
    {
      point.positions.resize(modelInfo_.actuatedDofNum);
      point.velocities.resize(modelInfo_.actuatedDofNum);
      point.effort.resize(modelInfo_.actuatedDofNum);
    }
    jointTrajectory_.points[0].time_from_start = rclcpp::Duration::from_seconds(0.0);
    jointTrajectory_.points[1].time_from_start = rclcpp::Duration::from_seconds(
      mrtDurationSeconds_);

    // Joint trajectory timer (not wall timer, as it uses system clock, not ROS one)
    jointTrajectoryTimer_ = rclcpp::create_timer(this, this->get_clock(), 
      rclcpp::Duration::from_seconds(mrtDurationSeconds_), 
      std::bind(&LeggedLocomotionMpcControllerNode::sendJointTrajectory, this));

    RCLCPP_INFO(this->get_logger(), "Legged Locomotion MPC Controller configured successfully!");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    LeggedLocomotionMpcControllerNode::on_activate(const State& state)
  {
    if(!waitForInitialState())
    {
      RCLCPP_ERROR(this->get_logger(),
        "No recent odometry or joint states, activation aborted!");
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }

    try
    {
      setupMpc();
    }
    catch(const std::exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "MPC setup error: %s", e.what());
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    mpcBenchmarkTimer_.reset();
    mrtBenchmarkTimer_.reset();
    commandActive_ = false;
    mpcFailed_ = false;
    jointTrajectoryPublisher_->on_activate();

    {
      std::lock_guard<std::mutex> lock(terrainMutex_);
      controllerRunning_ = true;

      // Terrain received during MPC setup
      if(terrainModelPtr_)
      {
        referenceManagerPtr_->updateTerrainModel(std::move(terrainModelPtr_));
      }
    }

    // Started after controllerRunning_ is set, otherwise MPC loop could exit immediately
    runMpc();

    RCLCPP_INFO(this->get_logger(), "Legged Locomotion MPC Controller activated successfully!");
    RCLCPP_INFO(this->get_logger(), "MPC/MRT loop activated!");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    LeggedLocomotionMpcControllerNode::on_deactivate(const State& state)
  {
    stopMpcThread();
    mpcFailed_ = false;

    jointTrajectoryPublisher_->on_deactivate();

    {
      // Keep current terrain for next activation, MPC thread is already stopped
      std::lock_guard<std::mutex> lock(terrainMutex_);
      if(!terrainModelPtr_ && referenceManagerPtr_)
      {
        terrainModelPtr_.reset(referenceManagerPtr_->getTerrainModel().clone());
      }
    }

    std::string returnString;
    returnString += "########################################################################";
    returnString += "\n### MPC Benchmarking";
    returnString += "\n###   Maximum : " + std::to_string(mpcBenchmarkTimer_.getMaxIntervalInMilliseconds()) + "[ms].";
    returnString += "\n###   Average : " + std::to_string(mpcBenchmarkTimer_.getAverageInMilliseconds()) + "[ms].\n";
    returnString += "########################################################################";
    returnString += "\n### WRT Benchmarking";
    returnString += "\n###   Maximum : " + std::to_string(mrtBenchmarkTimer_.getMaxIntervalInMilliseconds()) + "[ms].";
    returnString += "\n###   Average : " + std::to_string(mrtBenchmarkTimer_.getAverageInMilliseconds()) + "[ms].\n";

    RCLCPP_INFO(this->get_logger(), "%s", returnString.c_str());

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    LeggedLocomotionMpcControllerNode::on_shutdown(const State& state)
  {
    // Shutdown from active state does not call on_deactivate, controller would keep running
    if(state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
      on_deactivate(state);
    }

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    LeggedLocomotionMpcControllerNode::on_error(const State& state)
  {
    // Error can happen in any transition, controller can not keep running after it
    stopMpcThread();
    mpcFailed_ = false;

    if(jointTrajectoryPublisher_ && jointTrajectoryPublisher_->is_activated())
    {
      jointTrajectoryPublisher_->on_deactivate();
    }

    RCLCPP_ERROR(this->get_logger(), "Error in transition from %s state, controller stopped!",
      state.label().c_str());

    // Same as default on_error, node goes back to unconfigured state
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  bool LeggedLocomotionMpcControllerNode::waitForInitialState()
  {
    // Robot state has to be received during this activation
    jointStateReceived_ = false;
    odometryReceived_ = false;

    // MPC time starts at activation, states stamped before it get time 0
    mpcStartTime_ = this->get_clock()->now();
    currentObservation_.time = 0.0;

    // Steady clock, so waiting also ends when ROS time is not running
    const auto maxWaitDuration = std::chrono::seconds(1);
    const auto startTime = std::chrono::steady_clock::now();

    while(rclcpp::ok())
    {
      updateCurrentObservation();

      if(isStateFresh(this->get_clock()->now()))
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

  void LeggedLocomotionMpcControllerNode::setupMpc()
  {
    const std::string configDirectoryPath = this->get_parameter("config_directory_path").as_string();
    const std::string taskFilePath = configDirectoryPath + "/task.info";
    const std::string modelFilePath = configDirectoryPath + "/legged_model.info";
    const std::string urdfFilePath = this->get_parameter("urdf_path").as_string();

    // Destroy objects from previous activation, MPC uses data from interface
    mpcMrtPtr_.reset();
    mpcPtr_.reset();
    {
      std::lock_guard<std::mutex> lock(terrainMutex_);
      referenceManagerPtr_ = nullptr;
    }
    leggedInterfacePtr_.reset();

    // Initial system state and time from robot state messages (same time base as MRT loop)
    const vector_t initialSystemState = currentObservation_.state;
    const scalar_t initialTime = currentObservation_.time;

    std::unique_ptr<TerrainModel> initialTerrainModel;
    {
      std::lock_guard<std::mutex> lock(terrainMutex_);
      initialTerrainModel = std::move(terrainModelPtr_);
    }

    // Make plane terrain if terrain from subscriber is not available
    if(!initialTerrainModel)
    {      
      RCLCPP_WARN(this->get_logger(), 
        "Did not get any segmented plane terrain, using plane terrain with position [0,0,0] and rotation [0,0,0]!");
      
      const TerrainPlane initalPlane(vector3_t::Zero(), matrix3_t::Identity());

      initialTerrainModel =
        std::make_unique<PlanarTerrainModel>(initalPlane, modelSettings_.worldLinkName);
    }

    // Initialize loopshaping legged interface, 
    // legged interface and reference manager pointers
    leggedInterfacePtr_ = std::make_unique<LeggedInterface>(
      initialTime, initialSystemState, std::move(initialTerrainModel), taskFilePath,
      modelFilePath, urdfFilePath);

    {
      std::lock_guard<std::mutex> lock(terrainMutex_);
      referenceManagerPtr_ = &leggedInterfacePtr_->getLeggedReferenceManager();
    }
  
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

    // Make MPC MRT interface
    mpcMrtPtr_ = std::make_unique<MPC_MRT_Interface>(*mpcPtr_);

    const auto& modelInfo = leggedInterfacePtr_->floatingBaseModelInfo();
    
    const size_t endEffectorNum = modelInfo.numThreeDofContacts 
      + modelInfo.numSixDofContacts;
    const size_t standingMode = ((0x01 << (endEffectorNum)) - 1);
    const contact_flags_t standingFlags(standingMode);

    // Interface setup can take a long time, get newest robot state
    updateCurrentObservation();

    auto& weightCompensator = leggedInterfacePtr_->weightCompensator();

    const vector_t initialInput = weightCompensator.getInput(currentObservation_.state,
      standingFlags);

    // Get observation of the model
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
  }

  bool LeggedLocomotionMpcControllerNode::isStateFresh(const rclcpp::Time& currentTime) const
  {
    return jointStateReceived_ && odometryReceived_ &&
      (currentTime - lastJointStateTime_) <= stateTimeout_ &&
      (currentTime - lastOdometryTime_) <= stateTimeout_;
  }

  void LeggedLocomotionMpcControllerNode::updateCurrentObservation()
  {
    // Every message is handled separately, invalid one does not block others
    updateOdometry();
    updateJointStates();
    updateContacts();

    const rclcpp::Time newestStateTime = std::max(lastJointStateTime_, lastOdometryTime_);

    // Relative to MPC start, absolute ROS time (~1.7e9 s) in double has only ~2.4e-7 s resolution.
    // Observation time can not go backwards
    currentObservation_.time = std::max(currentObservation_.time,
      (newestStateTime - mpcStartTime_).seconds());
  }

  void LeggedLocomotionMpcControllerNode::updateOdometry()
  {
    rclcpp::MessageInfo msgInfo;

    auto& odometry = odometryMessage_;
    if(!odometrySubscriber_->take(odometry, msgInfo)) return;

    // Pose is expressed in header frame (world), twist in child frame (base)
    if(odometry.header.frame_id != modelSettings_.worldLinkName ||
       odometry.child_frame_id != modelSettings_.baseLinkName)
    {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Odometry has wrong base or world frame ID!");
      return;
    }

    const auto& basePose = odometry.pose.pose;
    const auto& baseTwist = odometry.twist.twist;

    vector6_t currentBasePose;

    currentBasePose(0) = basePose.position.x;
    currentBasePose(1) = basePose.position.y;
    currentBasePose(2) = basePose.position.z;

    Eigen::Quaterniond quaterion;

    tf2::fromMsg(basePose.orientation, quaterion);

    vector6_t currentBaseTwist;

    currentBaseTwist(0) = baseTwist.linear.x;
    currentBaseTwist(1) = baseTwist.linear.y;
    currentBaseTwist(2) = baseTwist.linear.z;
    currentBaseTwist(3) = baseTwist.angular.x;
    currentBaseTwist(4) = baseTwist.angular.y;
    currentBaseTwist(5) = baseTwist.angular.z;

    if(!currentBasePose.head<3>().allFinite() || !quaterion.coeffs().allFinite() ||
       !currentBaseTwist.allFinite())
    {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Ignored odometry with non-finite values");
      return;
    }

    const matrix3_t rotationMatrix = quaterion.normalized().toRotationMatrix();

    vector3_t eulerAngles = quaterion_euler_transforms::getEulerAnglesFromRotationMatrix(
      rotationMatrix);

    // Angles from rotation matrix are wrapped, keep them continuous with previous
    // observation (e.g. yaw crossing +-pi would jump by 2 pi)
    if(odometryReceived_)
    {
      const vector6_t previousBasePose = access_helper_functions::getBasePose(
        currentObservation_.state, modelInfo_);

      for(size_t i = 0; i < 3; ++i)
      {
        const scalar_t previousAngle = previousBasePose(3 + i);
        eulerAngles(i) = previousAngle + std::remainder(eulerAngles(i) - previousAngle, 2.0 * M_PI);
      }
    }

    currentBasePose(3) = eulerAngles(0);
    currentBasePose(4) = eulerAngles(1);
    currentBasePose(5) = eulerAngles(2);

    lastOdometryTime_ = odometry.header.stamp;
    odometryReceived_ = true;

    access_helper_functions::getBasePose(
      currentObservation_.state, modelInfo_) = currentBasePose;

    access_helper_functions::getBaseVelocity(
      currentObservation_.state, modelInfo_) = currentBaseTwist;
  }

  void LeggedLocomotionMpcControllerNode::updateJointStates()
  {
    rclcpp::MessageInfo msgInfo;

    auto& jointStates = jointStateMessage_;
    if(!jointStateSubscriber_->take(jointStates, msgInfo)) return;

    // Check if message has same amount of data (joint velocities are not used)
    if(jointStates.name.size() != jointStates.position.size() ||
       jointStates.name.size() != modelInfo_.actuatedDofNum)
    {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Ignored an invalid JointState message");
      return;
    }

    // Preallocated in on_configure
    auto& jointPositions = jointPositionsBuffer_;
    auto& jointUpdated = jointUpdatedBuffer_;
    std::fill(jointUpdated.begin(), jointUpdated.end(), false);

    for(size_t i = 0; i < jointStates.name.size(); ++i)
    {
      const auto jointIndexIt = jointNameIndexMap_.find(jointStates.name[i]);

      // Unknown or duplicated joint would leave some joint position unset
      if(jointIndexIt == jointNameIndexMap_.end() || jointUpdated[jointIndexIt->second])
      {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
          "Ignored JointState message with unknown or duplicated joint: %s",
          jointStates.name[i].c_str());
        return;
      }

      jointPositions[jointIndexIt->second] = jointStates.position[i];
      jointUpdated[jointIndexIt->second] = true;
    }

    if(!jointPositions.allFinite())
    {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Ignored JointState message with non-finite positions");
      return;
    }

    lastJointStateTime_ = jointStates.header.stamp;
    jointStateReceived_ = true;

    access_helper_functions::getJointPositions(currentObservation_.state,
      modelInfo_) = jointPositions;
  }

  void LeggedLocomotionMpcControllerNode::updateContacts()
  {
    rclcpp::MessageInfo msgInfo;

    auto& contacts = contactsMessage_;
    if(!contactsSubscriber_->take(contacts, msgInfo)) return;

    if(contacts.contacts.size() != endEffectorNum_)
    {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Ignored an invalid Contacts message");
      return;
    }

    contact_flags_t newContactFlags;

    // Preallocated in on_configure
    auto& contactUpdated = contactUpdatedBuffer_;
    std::fill(contactUpdated.begin(), contactUpdated.end(), false);

    for(const auto& currentContactMessage: contacts.contacts)
    {
      const auto contactIndexIt = contactFrameNameIndexMap_.find(
        currentContactMessage.header.frame_id);

      if(contactIndexIt == contactFrameNameIndexMap_.end() ||
         contactUpdated[contactIndexIt->second])
      {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
          "Ignored Contacts message with unknown or duplicated frame: %s",
          currentContactMessage.header.frame_id.c_str());
        return;
      }

      newContactFlags[contactIndexIt->second] = currentContactMessage.contact;
      contactUpdated[contactIndexIt->second] = true;
    }

    contactFlags_ = newContactFlags;
    newContactFlags_ = true;
  }

  void LeggedLocomotionMpcControllerNode::runMpc()
  {
    mpcThread_ = std::thread([this]()
    {
      while(controllerRunning_ && rclcpp::ok())
      {
        try 
        {
          const auto startTime = this->get_clock()->now();
          mpcBenchmarkTimer_.startTimer();
          mpcMrtPtr_->advanceMpc();
          mpcBenchmarkTimer_.endTimer();
          const auto endTime = this->get_clock()->now();
          const auto durationLeft = mpcDuration_ - (endTime - startTime);
          this->get_clock()->sleep_for(durationLeft);
        } 
        catch(const std::exception& e) 
        {
          controllerRunning_ = false;
          RCLCPP_ERROR(this->get_logger(), "MPC thread error : %s", e.what());

          // Deactivation joins this thread, so it is done by MRT timer
          mpcFailed_ = true;
        }
      }
    });
  }

  void LeggedLocomotionMpcControllerNode::stopMpcThread()
  {
    controllerRunning_ = false;
    if(mpcThread_.joinable()) mpcThread_.join();
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

    // NaN would pass through velocity limits of base planner
    if(!std::isfinite(baseTwist->twist.linear.x) || !std::isfinite(baseTwist->twist.linear.y) ||
       !std::isfinite(baseTwist->twist.linear.z) || !std::isfinite(baseTwist->twist.angular.z))
    {
      RCLCPP_ERROR(this->get_logger(),
        "Ignored base command with non-finite values");
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

      // Receive time, command stamps are often not set
      lastCommandTime_ = this->get_clock()->now();
      commandActive_ = true;
    }
  }

  void LeggedLocomotionMpcControllerNode::updateGaitParameters(
    const legged_locomotion_msgs::msg::GaitParameters::ConstSharedPtr gaitParameters)
  {
    const auto& phaseOffsets = gaitParameters->phase_offsets;
    const bool phaseOffsetsFinite = std::all_of(phaseOffsets.begin(), phaseOffsets.end(),
      [](scalar_t offset) {return std::isfinite(offset);});

    // Maximum stepping frequency is clamped by gait planner
    if(phaseOffsets.size() != endEffectorNum_ - 1 || !phaseOffsetsFinite ||
       !std::isfinite(gaitParameters->swing_ratio) ||
       gaitParameters->swing_ratio < 0 || gaitParameters->swing_ratio > 1 || 
       !std::isfinite(gaitParameters->stepping_frequency) ||
       gaitParameters->stepping_frequency < minimumSteppingFrequency_)
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

    const auto allOf = [](const std::vector<scalar_t>& values, auto predicate)
    {
      return std::all_of(values.begin(), values.end(), predicate);
    };

    // Same limits as in loadSwingPlannerDynamicSettings()
    const bool validValues =
      allOf(swingParameters->swing_heights,
        [](scalar_t height) {return std::isfinite(height) && height >= 0.0;}) &&
      allOf(swingParameters->phases,
        [](scalar_t phase) {return std::isfinite(phase) && phase > 0.0 && phase < 1.0;}) &&
      allOf(swingParameters->tangential_progresses,
        [](scalar_t progress) {return std::isfinite(progress) && progress >= 0.0 && progress <= 1.0;}) &&
      allOf(swingParameters->tangential_velocity_factors,
        [](scalar_t factor) {return std::isfinite(factor) && factor >= 0.0;});

    if(!validValues)
    {
      RCLCPP_ERROR(this->get_logger(),
        "Ignored SwingParameters message with values out of range");
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

    // Non-finite values would break MPC
    if(!baseWrench.allFinite())
    {
      RCLCPP_ERROR(this->get_logger(),
        "Ignored base external wrench with non-finite values");
      return;
    }

    if(leggedInterfacePtr_ && controllerRunning_)
    {
      // Same time base as observation
      leggedInterfacePtr_->disturbanceModule().updateDistrubance(
        (currentTime - mpcStartTime_).seconds(), baseWrench);
    }
  }

  void LeggedLocomotionMpcControllerNode::updateSegmentedTerrain(
    grid_map_msgs::msg::GridMap::UniquePtr gridMap)
  {
    // Runs in own callback group, in parallel to activation and deactivation.
    // Decomposition is slow, so terrainMutex_ is only held to get pipeline and hand over terrain
    std::shared_ptr<convex_plane_decomposition::PlaneDecompositionPipeline> decompositionPipeline;
    {
      std::lock_guard<std::mutex> lock(terrainMutex_);
      decompositionPipeline = decompositionPipelinePtr_;
    }

    if(!decompositionPipeline) return;

    // Terrain is not transformed, it has to be already in the world frame
    if(gridMap->header.frame_id != modelSettings_.worldLinkName)
    {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Ignored elevation map with frame %s, expected %s",
        gridMap->header.frame_id.c_str(), modelSettings_.worldLinkName.c_str());
      return;
    }

    // Make gridMap object
    grid_map::GridMap elevationMap;

    if(!GridMapRosConverter::fromMessage(*gridMap, elevationMap,
      {terrain_model::elevationLayerName}, true, false))
    {
      RCLCPP_ERROR(this->get_logger(),
        "Error during GridMapRosConverter::fromMessage()!");
      return;
    }

    // Keep previous terrain if the new one can not be used
    if(!elevationMap.get(terrain_model::elevationLayerName).array().isFinite().any())
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Ignored elevation map without any finite value");
      return;
    }

    std::unique_ptr<terrain_model::SegmentedPlanesTerrainModel> terrainModel;
    try
    {
      decompositionPipeline->update(std::move(elevationMap),
        terrain_model::elevationLayerName);

      auto planarTerrain = decompositionPipeline->movePlanarTerrain();

      if(planarTerrain.planarRegions.empty())
      {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
          "Ignored elevation map without any planar region");
        return;
      }

      terrainModel = std::make_unique<terrain_model::SegmentedPlanesTerrainModel>(
        std::move(planarTerrain), modelSettings_.worldLinkName);
    }
    catch(const std::exception& e)
    {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Ignored elevation map, plane decomposition failed: %s", e.what());
      return;
    }

    std::lock_guard<std::mutex> lock(terrainMutex_);

    if(referenceManagerPtr_ && controllerRunning_)
    {
      referenceManagerPtr_->updateTerrainModel(std::move(terrainModel));
    }
    else
    {
      // Used during next MPC setup
      terrainModelPtr_ = std::move(terrainModel);
    }
  }

  void LeggedLocomotionMpcControllerNode::sendJointTrajectory()
  {
    // MPC thread can not deactivate node by itself (it would join itself)
    if(mpcFailed_.exchange(false))
    {
      RCLCPP_ERROR(this->get_logger(), "MPC loop deactivation!");
      this->deactivate();
      return;
    }

    if(!mpcMrtPtr_ || !controllerRunning_) return;

    try
    {
      mrtBenchmarkTimer_.startTimer();

      updateCurrentObservation();

      const rclcpp::Time currentTime = this->get_clock()->now();

      if(!isStateFresh(currentTime))
      {
        RCLCPP_ERROR(this->get_logger(),
          "Robot state is older than state_timeout, MPC loop deactivation!");
        this->deactivate();
        return;
      }

      // Stop robot if velocity commands stopped
      if(commandActive_ && (currentTime - lastCommandTime_) > velocityTimeout_)
      {
        RCLCPP_WARN(this->get_logger(),
          "No base velocity command for longer than velocity_timeout, stopping!");
        referenceManagerPtr_->updateCommand(
          planners::BaseTrajectoryPlanner::BaseReferenceCommand{});
        commandActive_ = false;
      }

      // Update contact flags only when new message was received
      if(newContactFlags_)
      {
        referenceManagerPtr_->updateContactFlags(contactFlags_);
        newContactFlags_ = false;
      }

      mpcMrtPtr_->setCurrentObservation(currentObservation_);

      mpcMrtPtr_->updatePolicy();

      const auto& policy = mpcMrtPtr_->getPolicy();

      // MPC thread can stall without exception, old policy can not be used for long
      if(currentObservation_.time - policy.timeTrajectory_.front() > policyTimeout_)
      {
        RCLCPP_ERROR(this->get_logger(),
          "MPC policy is older than policy_timeout, MPC loop deactivation!");
        this->deactivate();
        return;
      }

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
      const auto& timeTrajectory = policy.timeTrajectory_;
      const auto& stateTrajectory = policy.stateTrajectory_;

      const auto nextState = LinearInterpolation::interpolate(nextTime,
        timeTrajectory, stateTrajectory);

      // Same controller as for the first point (with feedback), current deviation from
      // optimal state is propagated to the next point, otherwise feedback would treat
      // planned motion between points as tracking error
      const vector_t nextFeedbackState = nextState + (currentState - optimizedState);
      const vector_t nextInput = policy.controllerPtr_->computeInput(nextTime,
        nextFeedbackState);

      setTrajectoryPoint(0, optimizedState, optimizedInput);
      setTrajectoryPoint(1, nextState, nextInput);

      mrtBenchmarkTimer_.endTimer();

      // Publish
      if(jointTrajectoryPublisher_->is_activated())
      {
        jointTrajectoryPublisher_->publish(jointTrajectory_);
      }
    }
    catch(const std::exception& e)
    {
      // Exception would leave the executor and terminate the whole process
      RCLCPP_ERROR(this->get_logger(), "MRT loop error: %s, MPC loop deactivation!", e.what());
      this->deactivate();
    }
  }

  void LeggedLocomotionMpcControllerNode::setTrajectoryPoint(size_t pointIndex,
    const vector_t& state, const vector_t& input)
  {
    // Write directly into preallocated message vectors, no allocations
    auto& point = jointTrajectory_.points[pointIndex];
    const size_t jointNum = modelInfo_.actuatedDofNum;

    Eigen::Map<vector_t>(point.positions.data(), jointNum) =
      access_helper_functions::getJointPositions(state, modelInfo_);

    Eigen::Map<vector_t>(point.velocities.data(), jointNum) =
      access_helper_functions::getJointVelocities(input, modelInfo_);

    Eigen::Map<vector_t>(point.effort.data(), jointNum) =
      leggedInterfacePtr_->torqueApproximator().getValue(state, input);
  }
} // namespace legged_locomotion_mpc_ros2
