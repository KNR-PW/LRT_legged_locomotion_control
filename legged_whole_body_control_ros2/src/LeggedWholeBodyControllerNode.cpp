#include <legged_whole_body_control_ros2/LeggedWholeBodyControllerNode.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>

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

    // Maximum age of odometry and joint states messages [s], controller is deactivated after that
    this->declare_parameter("odometry_timeout", 0.1);
    this->declare_parameter("joint_state_timeout", 0.1);

    // Maximum duration between external wrench messages [s], wrench is zeroed after that
    this->declare_parameter("wrench_timeout", 0.1);

    // Number of not solved WBC problems in a row, controller is deactivated after that
    this->declare_parameter("max_consecutive_failures", 10);

    // Maximum age of MPC policy [s] (from its start time), WBC is standing after that.
    // Has to be longer than MPC period + MPC solve time and shorter than MPC horizon
    this->declare_parameter("policy_timeout", 0.5);

    // Contact sensors correct planned contact flags of the policy (late and early touchdowns)
    this->declare_parameter("use_contact_sensors", true);

    // Maximum time without contacts messages [s] (receive time), planned contact flags are used after that
    this->declare_parameter("contacts_timeout", 0.1);

    // Same clock type as node clock, times from different clocks can not be compared
    const rclcpp::Time zeroTime(0, 0, this->get_clock()->get_clock_type());
    lastJointStateTime_ = zeroTime;
    lastOdometryTime_ = zeroTime;
    wbcStartTime_ = zeroTime;
    lastWrenchTime_ = zeroTime;
    lastContactsReceiveTime_ = zeroTime;

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
      RCLCPP_ERROR(this->get_logger(), "Error while loading legged_wbc.info file: %s", e.what());
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    if(this->get_parameter("type").as_string() != "weighted")
    {
      RCLCPP_ERROR(this->get_logger(), "Error: Only \"weighted\" type supported!");
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    const scalar_t odometryTimeout = this->get_parameter("odometry_timeout").as_double();
    const scalar_t jointStateTimeout = this->get_parameter("joint_state_timeout").as_double();
    const scalar_t wrenchTimeout = this->get_parameter("wrench_timeout").as_double();
    const int64_t maxConsecutiveFailures = this->get_parameter("max_consecutive_failures").as_int();
    const scalar_t policyTimeout = this->get_parameter("policy_timeout").as_double();
    const scalar_t contactsTimeout = this->get_parameter("contacts_timeout").as_double();

    if(!(odometryTimeout > 0.0) || !(jointStateTimeout > 0.0) || !(wrenchTimeout > 0.0) ||
       maxConsecutiveFailures < 1 || !(policyTimeout > 0.0) || !(contactsTimeout > 0.0))
    {
      RCLCPP_ERROR(this->get_logger(),
        "Parameters odometry_timeout, joint_state_timeout, wrench_timeout, "
        "max_consecutive_failures, policy_timeout and contacts_timeout have to be positive!");
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    odometryTimeout_ = rclcpp::Duration::from_seconds(odometryTimeout);
    jointStateTimeout_ = rclcpp::Duration::from_seconds(jointStateTimeout);
    wrenchTimeout_ = rclcpp::Duration::from_seconds(wrenchTimeout);
    maxConsecutiveFailures_ = static_cast<size_t>(maxConsecutiveFailures);
    policyTimeout_ = policyTimeout;
    useContactSensors_ = this->get_parameter("use_contact_sensors").as_bool();
    contactsTimeout_ = rclcpp::Duration::from_seconds(contactsTimeout);

    const std::string urdfFilePath = this->get_parameter("urdf_path").as_string();

    // URDF is parsed once, WBC is created from this interface on every activation
    try
    {
      pinocchioInterfacePtr_ = std::make_unique<PinocchioInterface>(
        createPinocchioInterfaceFromUrdfFile(urdfFilePath, wbcSettings_.baseLinkName));

      modelInfo_ = createFloatingBaseModelInfo(*pinocchioInterfacePtr_,
        wbcSettings_.endEffectorThreeDofNames, wbcSettings_.endEffectorSixDofNames);
    }
    catch(const std::exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "Error while creating robot model from URDF: %s", e.what());
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    // Resize current measured observation
    currentMeasuredObservation_.state = vector_t::Zero(modelInfo_.stateDim);
    currentMeasuredObservation_.input = vector_t::Zero(modelInfo_.inputDim);

    // Resize standing and desired references
    standingState_ = vector_t::Zero(modelInfo_.stateDim);
    standingInput_ = vector_t::Zero(modelInfo_.inputDim);
    previousDesiredState_ = vector_t::Zero(modelInfo_.stateDim);
    previousDesiredInput_ = vector_t::Zero(modelInfo_.inputDim);
    desiredState_ = vector_t::Zero(modelInfo_.stateDim);
    desiredInput_ = vector_t::Zero(modelInfo_.inputDim);
    nextDesiredState_ = vector_t::Zero(modelInfo_.stateDim);
    nextDesiredInput_ = vector_t::Zero(modelInfo_.inputDim);

    endEffectorNum_ = modelInfo_.numThreeDofContacts + modelInfo_.numSixDofContacts;

    // MRT of the whole body controller, policy dimensions are checked with the model
    try
    {
      policyEvaluatorPtr_ = std::make_unique<MpcPolicyEvaluator>(modelInfo_.stateDim,
        modelInfo_.inputDim, endEffectorNum_);
    }
    catch(const std::exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "Error while creating MPC policy evaluator: %s", e.what());
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    contactFlagsFusionPtr_ = std::make_unique<ContactFlagsFusion>(endEffectorNum_);
    timesToTouchdown_.assign(endEffectorNum_, 0.0);

    // Contact frames in WBC end effector order, names from previous configuration can be different
    contactFrameNameIndexMap_.clear();

    for(size_t i = 0; i < wbcSettings_.endEffectorThreeDofNames.size(); ++i)
    {
      contactFrameNameIndexMap_[wbcSettings_.endEffectorThreeDofNames[i]] = i;
    }

    for(size_t i = 0; i < wbcSettings_.endEffectorSixDofNames.size(); ++i)
    {
      contactFrameNameIndexMap_[wbcSettings_.endEffectorSixDofNames[i]] =
        wbcSettings_.endEffectorThreeDofNames.size() + i;
    }

    const auto& model = pinocchioInterfacePtr_->getModel();

    jointNames_ = model.names;

    // Remove "universe" and "root_joint" joints from joint names
    jointNames_.erase(std::remove(jointNames_.begin(), jointNames_.end(), "universe"), jointNames_.end());
    jointNames_.erase(std::remove(jointNames_.begin(), jointNames_.end(), "root_joint"), jointNames_.end()); 

    jointNameIndexMap_.clear();

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

    // MPC policy subscriber, reliable as the publisher (large message is fragmented,
    // best effort would drop it when any fragment is lost)
    const std::string policyTopic = "/mpc_policy";
    policySubscriber_ = this->create_subscription<legged_locomotion_msgs::msg::MpcPolicy>(
      policyTopic, rclcpp::QoS(1).reliable().durability_volatile(),
      [](const legged_locomotion_msgs::msg::MpcPolicy::ConstSharedPtr) {}, subscriptionOptions);

    // Joint states subscriber
    const std::string jointSatesTopic = "/joint_states";
    jointStateSubscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
      jointSatesTopic, qos, 
      [](const sensor_msgs::msg::JointState::ConstSharedPtr) {}, subscriptionOptions);
    
    // Base pose and twist subscriber
    const std::string odometryTopic = "/odom";
    odometrySubscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odometryTopic, qos,
      [](const nav_msgs::msg::Odometry::ConstSharedPtr) {}, subscriptionOptions);

    // Contact sensors subscriber
    if(useContactSensors_)
    {
      const std::string contactsTopic = "/contacts";
      contactsSubscriber_ = this->create_subscription<contact_msgs::msg::Contacts>(
        contactsTopic, qos,
        [](const contact_msgs::msg::Contacts::ConstSharedPtr) {}, subscriptionOptions);
    }
    else
    {
      contactsSubscriber_.reset();
    }

    // Base external wrench subsciber
    const std::string baseWrenchTopic = "/base_external_wrench";
    baseWrenchSubscriber_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
      baseWrenchTopic, QoS(1).reliable().keep_last(1), 
      std::bind(&LeggedWholeBodyControllerNode::updateExternalWrench, this, std::placeholders::_1));
    
    // Joint trajectory publisher, same QoS as MRT control in MPC node. Only newest trajectory
    // matters. Volatile, so subscriber started later does not get old trajectory
    const std::string jointTrajectoryTopic = "/joint_trajectory";
    jointTrajectoryPublisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      jointTrajectoryTopic, QoS(1).reliable().durability_volatile());

    /**
     * Create timer for joint trajectory commands
     */

    // Get duration 
    wbcDurationSeconds_ = 1.0 / wbcSettings_.desiredFrequency;

    // Preallocate joint trajectory message, WBC loop only writes point values
    jointTrajectory_ = trajectory_msgs::msg::JointTrajectory();

    // Start trajectory now (time 0 seconds 0 nanoseconds)
    jointTrajectory_.header.stamp.sec = 0;
    jointTrajectory_.header.stamp.nanosec = 0;
    jointTrajectory_.joint_names = jointNames_;

    jointTrajectory_.points.resize(1);
    jointTrajectory_.points[0].time_from_start = rclcpp::Duration::from_seconds(0.0);
    jointTrajectory_.points[0].positions.resize(modelInfo_.actuatedDofNum);
    jointTrajectory_.points[0].velocities.resize(modelInfo_.actuatedDofNum);
    jointTrajectory_.points[0].effort.resize(modelInfo_.actuatedDofNum);

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
    // WBC starts from measured state, it has to be known
    if(!waitForInitialState())
    {
      RCLCPP_ERROR(this->get_logger(),
        "No recent odometry or joint states, activation aborted!");
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }

    try
    {
      setupWbc();
    }
    catch(const std::exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "WBC setup error: %s", e.what());
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    // New WBC starts without external wrench
    wrenchActive_ = false;
    consecutiveFailures_ = 0;

    // WBC stands until MPC policy is received
    followingPolicy_ = false;

    // Contacts have to be received during this activation, planned flags are used until then
    contactsReceived_ = false;

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

    // Non-finite values would make every WBC problem unsolvable
    if(!baseWrench.allFinite())
    {
      RCLCPP_ERROR(this->get_logger(),
        "Ignored base external wrench with non-finite values");
      return;
    }

    if(wbcPtr_ && controllerRunning_)
    {
      // Same time base as measured observation
      wbcPtr_->updateExternalWrench((currentTime - wbcStartTime_).seconds(), baseWrench);

      // Receive time, same as base velocity command in MPC node
      lastWrenchTime_ = this->get_clock()->now();
      wrenchActive_ = true;
    }
  }
  
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void LeggedWholeBodyControllerNode::sendJointTrajectory()
  {
    if(!wbcPtr_ || !controllerRunning_) return;

    try
    {
      wbcBenchmarkTimer_.startTimer();

      updateMeasuredObservation();

      const rclcpp::Time currentTime = this->get_clock()->now();

      if(!isOdometryFresh(currentTime))
      {
        RCLCPP_ERROR(this->get_logger(),
          "Odometry is older than odometry_timeout, WBC loop deactivation!");
        this->deactivate();
        return;
      }

      if(!isJointStateFresh(currentTime))
      {
        RCLCPP_ERROR(this->get_logger(),
          "Joint states are older than joint_state_timeout, WBC loop deactivation!");
        this->deactivate();
        return;
      }

      // Stop compensating external wrench if wrench messages stopped
      if(wrenchActive_ && (currentTime - lastWrenchTime_) > wrenchTimeout_)
      {
        RCLCPP_WARN(this->get_logger(),
          "No base external wrench for longer than wrench_timeout, wrench set to zero!");
        wbcPtr_->updateExternalWrench((currentTime - wbcStartTime_).seconds(), vector6_t::Zero());
        wrenchActive_ = false;
      }

      updatePolicy();
      updateContacts();

      // Policy is evaluated at time of measured state, as by MRT in MPC node
      const scalar_t time = currentMeasuredObservation_.time;

      if(isPolicyValid(time))
      {
        if(!followingPolicy_)
        {
          RCLCPP_INFO(this->get_logger(), "WBC is following MPC policy!");
          followingPolicy_ = true;

          // Robot was standing, every end effector in swing lifts off now
          contactFlagsFusionPtr_->reset(time, contact_flags_t((size_t(1) << endEffectorNum_) - 1));
        }

        const auto& measuredState = currentMeasuredObservation_.state;

        // WBC computes desired accelerations from two last desired points,
        // both are taken from the same policy, so policy updates do not make jumps in them
        policyEvaluatorPtr_->evaluate(time - wbcDurationSeconds_, measuredState,
          previousDesiredState_, previousDesiredInput_);
        policyEvaluatorPtr_->evaluate(time, measuredState, desiredState_, desiredInput_);

        // Not after the policy end (MRT warns about it), the last point is held there anyway
        const scalar_t nextTime = std::min(time + wbcDurationSeconds_,
          policyEvaluatorPtr_->getFinalTime());
        policyEvaluatorPtr_->evaluate(nextTime, measuredState, nextDesiredState_, nextDesiredInput_);

        contactFlags_ = getPolicyContactFlags(time, currentTime);
        terrainNormals_ = policyEvaluatorPtr_->getTerrainNormals(time);
      }
      else
      {
        // Robot stops where it is, when policy is lost
        if(followingPolicy_)
        {
          RCLCPP_WARN(this->get_logger(),
            "MPC policy is older than policy_timeout or ended, WBC is standing!");
          followingPolicy_ = false;
          setStandingReference();
        }

        previousDesiredState_ = standingState_;
        previousDesiredInput_ = standingInput_;
        desiredState_ = standingState_;
        desiredInput_ = standingInput_;
        nextDesiredState_ = standingState_;
        nextDesiredInput_ = standingInput_;

        // All end effectors in contact, last terrain normals are kept
        contactFlags_ = contact_flags_t((size_t(1) << endEffectorNum_) - 1);
      }

      wbcPtr_->updateDesired(time - wbcDurationSeconds_, previousDesiredState_, previousDesiredInput_);
      wbcPtr_->updateDesired(time, desiredState_, desiredInput_);

      wbcPtr_->updateMeasured(time, currentMeasuredObservation_.state,
        currentMeasuredObservation_.input);

      wbcPtr_->updateContactFlags(time, contactFlags_);

      wbcPtr_->updateTerrainNormals(time, terrainNormals_);

      const bool solved = wbcPtr_->calculate(time);

      // WBC keeps previous result when problem is not solved (zeros at start),
      // so nothing is sent and joint controller keeps previous command
      if(!solved)
      {
        wbcBenchmarkTimer_.endTimer();

        if(++consecutiveFailures_ >= maxConsecutiveFailures_)
        {
          RCLCPP_ERROR(this->get_logger(),
            "WBC problem not solved %zu times in a row, WBC loop deactivation!",
            consecutiveFailures_);
          this->deactivate();
          return;
        }

        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
          "WBC problem not solved, joint trajectory not sent!");
        return;
      }

      consecutiveFailures_ = 0;

      // Write directly into preallocated message vectors, no allocations
      auto& point = jointTrajectory_.points[0];
      const size_t jointNum = modelInfo_.actuatedDofNum;

      Eigen::Map<vector_t> velocities(point.velocities.data(), jointNum);
      Eigen::Map<vector_t> positions(point.positions.data(), jointNum);

      if(followingPolicy_)
      {
        // Planned joint state one WBC period ahead (as second point of MRT joint trajectory)
        velocities = access_helper_functions::getJointVelocities(nextDesiredInput_, modelInfo_);
        positions = access_helper_functions::getJointPositions(nextDesiredState_, modelInfo_);
      }
      else
      {
        // Standing joint state moved by WBC joint accelerations
        velocities = access_helper_functions::getJointVelocities(desiredInput_, modelInfo_)
          + wbcPtr_->getJointAcceleration() * wbcDurationSeconds_;
        positions = access_helper_functions::getJointPositions(desiredState_, modelInfo_)
          + velocities * wbcDurationSeconds_;
      }

      Eigen::Map<vector_t>(point.effort.data(), jointNum) = wbcPtr_->getJointTorque();

      wbcBenchmarkTimer_.endTimer();

      // Publish 
      if(jointTrajectoryPublisher_->is_activated())
      {
        jointTrajectoryPublisher_->publish(jointTrajectory_);
      }
    }
    catch(const std::exception& e)
    {
      // Exception would leave the executor and terminate the whole process
      RCLCPP_ERROR(this->get_logger(), "WBC loop error: %s, WBC loop deactivation!", e.what());
      this->deactivate();
    }
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void LeggedWholeBodyControllerNode::setupWbc()
  {
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
        const std::string errorMessage = "Error while loading weighted_wbc_settings from legged_wbc.info file: "
          + std::string(e.what());
        throw std::runtime_error(errorMessage);
      }
    }
    else
    {
      const std::string errorMessage = "Error: Only \"weighted\" type supported!";
      throw std::runtime_error(errorMessage);
    }

    // WBC copies the interface, so the one from on_configure is not changed
    wbcPtr_ = std::make_unique<WeightedWbc>(*pinocchioInterfacePtr_, modelInfo_, wbcSettings_, weights);

    // WBC setup can take a long time, get newest robot state
    updateMeasuredObservation();

    // Policy from previous activation has times relative to previous WBC start time
    policyEvaluatorPtr_->reset();

    // WBC stands on flat terrain until MPC policy is received
    contactFlags_ = contact_flags_t((size_t(1) << endEffectorNum_) - 1);
    terrainNormals_.assign(endEffectorNum_, vector3_t{0.0, 0.0, 1.0});
    setStandingReference();

    const scalar_t time = currentMeasuredObservation_.time;

    wbcPtr_->updateDesired(time, standingState_, standingInput_);

    wbcPtr_->updateMeasured(time, currentMeasuredObservation_.state,
      currentMeasuredObservation_.input);

    wbcPtr_->updateContactFlags(time, contactFlags_);

    wbcPtr_->updateTerrainNormals(time, terrainNormals_);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void LeggedWholeBodyControllerNode::setStandingReference()
  {
    const contact_flags_t standingFlags((size_t(1) << endEffectorNum_) - 1);

    // Robot stays where it is now
    standingState_ = currentMeasuredObservation_.state;
    access_helper_functions::getBaseVelocity(standingState_, modelInfo_).setZero();

    // Weight is distributed equally on all end effectors, joints do not move
    standingInput_ = weightCompensatingInput(modelInfo_, standingFlags);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void LeggedWholeBodyControllerNode::updatePolicy()
  {
    rclcpp::MessageInfo msgInfo;

    auto& policyMessage = policyMessage_;
    if(!policySubscriber_->take(policyMessage, msgInfo)) return;

    // Policy is not transformed, it has to be in the world frame
    if(policyMessage.header.frame_id != wbcSettings_.worldLinkName)
    {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Ignored MPC policy with frame %s, expected %s",
        policyMessage.header.frame_id.c_str(), wbcSettings_.worldLinkName.c_str());
      return;
    }

    // Policy times are relative to MPC start time (header stamp), they are moved to WBC time
    const rclcpp::Time mpcStartTime(policyMessage.header.stamp, this->get_clock()->get_clock_type());
    const scalar_t timeOffset = (mpcStartTime - wbcStartTime_).seconds();

    std::string error;
    if(!policyEvaluatorPtr_->readPolicy(policyMessage, timeOffset, error))
    {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Ignored invalid MPC policy: %s", error.c_str());
    }
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  bool LeggedWholeBodyControllerNode::isPolicyValid(scalar_t time) const
  {
    return policyEvaluatorPtr_->hasPolicy() &&
      (time - policyEvaluatorPtr_->getStartTime()) <= policyTimeout_ &&
      time <= policyEvaluatorPtr_->getFinalTime();
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void LeggedWholeBodyControllerNode::updateContacts()
  {
    if(!contactsSubscriber_) return;

    rclcpp::MessageInfo msgInfo;

    contact_msgs::msg::Contacts contacts;
    if(!contactsSubscriber_->take(contacts, msgInfo)) return;

    if(contacts.contacts.size() != endEffectorNum_)
    {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Ignored an invalid Contacts message");
      return;
    }

    contact_flags_t newContactFlags;
    std::vector<bool> contactUpdated(endEffectorNum_, false);

    for(const auto& contact: contacts.contacts)
    {
      const auto contactIndexIt = contactFrameNameIndexMap_.find(contact.header.frame_id);

      // Unknown or duplicated frame would leave some contact flag unset
      if(contactIndexIt == contactFrameNameIndexMap_.end() || contactUpdated[contactIndexIt->second])
      {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
          "Ignored Contacts message with unknown or duplicated frame: %s",
          contact.header.frame_id.c_str());
        return;
      }

      newContactFlags[contactIndexIt->second] = contact.contact;
      contactUpdated[contactIndexIt->second] = true;
    }

    measuredContactFlags_ = newContactFlags;

    // Receive time, contact stamps are often not set
    lastContactsReceiveTime_ = this->get_clock()->now();
    contactsReceived_ = true;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  bool LeggedWholeBodyControllerNode::isContactsFresh(const rclcpp::Time& currentTime) const
  {
    return contactsReceived_ && (currentTime - lastContactsReceiveTime_) <= contactsTimeout_;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  contact_flags_t LeggedWholeBodyControllerNode::getPolicyContactFlags(scalar_t time,
    const rclcpp::Time& currentTime)
  {
    const contact_flags_t plannedFlags = policyEvaluatorPtr_->getContactFlags(time);

    if(!useContactSensors_) return plannedFlags;

    if(!isContactsFresh(currentTime))
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "No contacts message for longer than contacts_timeout, planned contact flags are used!");

      // Lift off times are not known when contacts are back, swing is assumed to start then
      contactFlagsFusionPtr_->reset(time, plannedFlags);
      return plannedFlags;
    }

    policyEvaluatorPtr_->getTimesToTouchdown(time, timesToTouchdown_);
    return contactFlagsFusionPtr_->update(time, plannedFlags, measuredContactFlags_,
      timesToTouchdown_);
  }
  
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  bool LeggedWholeBodyControllerNode::waitForInitialState()
  {
    // Robot state has to be received during this activation
    jointStateReceived_ = false;
    odometryReceived_ = false;

    // WBC time starts at activation, states stamped before it get time 0
    wbcStartTime_ = this->get_clock()->now();
    currentMeasuredObservation_.time = 0.0;

    // Steady clock, so waiting also ends when ROS time is not running
    const auto maxWaitDuration = std::chrono::seconds(1);
    const auto startTime = std::chrono::steady_clock::now();

    while(rclcpp::ok())
    {
      updateMeasuredObservation();

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

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  bool LeggedWholeBodyControllerNode::isStateFresh(const rclcpp::Time& currentTime) const
  {
    return isOdometryFresh(currentTime) && isJointStateFresh(currentTime);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  bool LeggedWholeBodyControllerNode::isOdometryFresh(const rclcpp::Time& currentTime) const
  {
    return odometryReceived_ && (currentTime - lastOdometryTime_) <= odometryTimeout_;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  bool LeggedWholeBodyControllerNode::isJointStateFresh(const rclcpp::Time& currentTime) const
  {
    return jointStateReceived_ && (currentTime - lastJointStateTime_) <= jointStateTimeout_;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void LeggedWholeBodyControllerNode::updateMeasuredObservation()
  {
    // Every message is handled separately, invalid one does not block others
    updateOdometry();
    updateJointStates();

    const rclcpp::Time newestStateTime = std::max(lastJointStateTime_, lastOdometryTime_);

    // Relative to WBC start, absolute ROS time (~1.7e9 s) in double has only ~2.4e-7 s resolution.
    // Observation time can not go backwards
    currentMeasuredObservation_.time = std::max(currentMeasuredObservation_.time,
      (newestStateTime - wbcStartTime_).seconds());
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void LeggedWholeBodyControllerNode::updateOdometry()
  {
    rclcpp::MessageInfo msgInfo;

    nav_msgs::msg::Odometry odometry;
    if(!odometrySubscriber_->take(odometry, msgInfo)) return;

    // Pose is expressed in header frame (world), twist in child frame (base)
    if(odometry.header.frame_id != wbcSettings_.worldLinkName ||
       odometry.child_frame_id != wbcSettings_.baseLinkName)
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

    // Zero quaternion can not be normalized
    if(!currentBasePose.head<3>().allFinite() || !quaterion.coeffs().allFinite() ||
       quaterion.norm() < 1e-6 || !currentBaseTwist.allFinite())
    {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Ignored odometry with non-finite values or zero quaternion");
      return;
    }

    const matrix3_t rotationMatrix = quaterion.normalized().toRotationMatrix();

    // Wrapped angles are fine, WBC uses only rotation matrices made from them
    const vector3_t eulerAngles = quaterion_euler_transforms::getEulerAnglesFromRotationMatrix(
      rotationMatrix);

    currentBasePose(3) = eulerAngles(0);
    currentBasePose(4) = eulerAngles(1);
    currentBasePose(5) = eulerAngles(2);

    lastOdometryTime_ = odometry.header.stamp;
    odometryReceived_ = true;

    access_helper_functions::getBasePose(
      currentMeasuredObservation_.state, modelInfo_) = currentBasePose;

    access_helper_functions::getBaseVelocity(
      currentMeasuredObservation_.state, modelInfo_) = currentBaseTwist;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void LeggedWholeBodyControllerNode::updateJointStates()
  {
    rclcpp::MessageInfo msgInfo;

    sensor_msgs::msg::JointState jointStates;
    if(!jointStateSubscriber_->take(jointStates, msgInfo)) return;

    // Check if message has same amount of data
    if(jointStates.name.size() != jointStates.position.size() ||
       jointStates.name.size() != jointStates.velocity.size() ||
       jointStates.name.size() != modelInfo_.actuatedDofNum)
    {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Ignored an invalid JointState message");
      return;
    }

    vector_t jointPositions = vector_t(modelInfo_.actuatedDofNum);
    vector_t jointVelocities = vector_t(modelInfo_.actuatedDofNum);
    std::vector<bool> jointUpdated(modelInfo_.actuatedDofNum, false);

    for(size_t i = 0; i < jointStates.name.size(); ++i)
    {
      const auto jointIndexIt = jointNameIndexMap_.find(jointStates.name[i]);

      // Unknown or duplicated joint would leave some joint state unset
      if(jointIndexIt == jointNameIndexMap_.end() || jointUpdated[jointIndexIt->second])
      {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
          "Ignored JointState message with unknown or duplicated joint: %s",
          jointStates.name[i].c_str());
        return;
      }

      jointPositions[jointIndexIt->second] = jointStates.position[i];
      jointVelocities[jointIndexIt->second] = jointStates.velocity[i];
      jointUpdated[jointIndexIt->second] = true;
    }

    lastJointStateTime_ = jointStates.header.stamp;
    jointStateReceived_ = true;

    access_helper_functions::getJointPositions(currentMeasuredObservation_.state,
      modelInfo_) = jointPositions;

    // Joint velocities are part of the input
    access_helper_functions::getJointVelocities(currentMeasuredObservation_.input,
      modelInfo_) = jointVelocities;
  }
} // namespace legged_whole_body_control_ros2