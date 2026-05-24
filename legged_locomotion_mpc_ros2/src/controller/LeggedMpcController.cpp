#include <legged_locomotion_mpc_ros2/controller/LeggedMpcController.hpp>

#include <floating_base_model/AccessHelperFunctions.hpp>

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
    this->declare_parameter("joint_trajectory_topic", "~/joint_forward_trajectory");

    configDirectoryPath_ = this->get_parameter("config_directory_path").as_string();
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

    const auto model = interface.getModel();

    std::vector<std::string> jointNames = model.names;

    // Remove "universe" and "root_joint" joints from joint names
    jointNames.erase(std::remove(jointNames.begin(), jointNames.end(), "universe"), jointNames.end());
    jointNames.erase(std::remove(jointNames.begin(), jointNames.end(), "root_joint"), jointNames.end()); 

    for(size_t i = 0; i < jointNames.size(); ++i)
    {
      jointNameIndexMap_[jointNames[i]] = i;
    }

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
      contactFlagsTopic, QoS(1).best_effort().keep_last(1), 
      std::bind(&LeggedMpcController::updateContactFlags, this, std::placeholders::_1));
    
    // Gait parameters subscriber
    const std::string gaitParametersTopic = this->get_parameter("gait_parameters_topic").as_string();
    gaitParametersSubscriber_ = this->create_subscription<legged_locomotion_msgs::msg::GaitParameterss>(
      gaitParametersTopic, QoS(1).best_effort().keep_last(1), 
      std::bind(&LeggedMpcController::updateGaitParameters, this, std::placeholders::_1));

    // Joint trajectory publisher
    const std::string jointTrajectoryTopic = this->get_parameter("joint_trajectory_topic").as_string();
    jointTrajectoryPublisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      jointTrajectoryTopic, SystemDefaultsQoS());
  }

  void LeggedMpcController::updateCommand(
    const geometry_msgs::msg::TwistStamped::ConstSharedPtr baseTwist)
  {
    // Check if message has good base link in header
    if(baseTwist->header.frame_id != modelSettings_.baseLinkName) return;

    planners::BaseTrajectoryPlanner::BaseReferenceCommand command;
    command.baseHeadingVelocity = baseTwist->twist.linear.x;
    command.baseLateralVelocity = baseTwist->twist.linear.y;
    command.baseVerticalVelocity = baseTwist->twist.linear.z;
    command.yawRate = baseTwist->twist.angular.z;

    if(referenceManagerPtr_)
    {
      referenceManagerPtr_->updateCommand(command);
    }
  }

  void LeggedMpcController::updateJointStates(
    const sensor_msgs::msg::JointState::ConstSharedPtr jointStates)
  {
    // Check if message has same amount of data
    if(jointStates->name.size() != jointStates->position.size() || 
       jointStates->name.size() != jointStates->velocity.size())
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

    lastJointStateTime_.seconds = jointStates->header.stamp.sec;
    lastJointStateTime_.nanoseconds = jointStates->header.stamp.nanosec;
    
    auto& currentState = currentSystemObservation_.state;
    auto& currentInput = currentSystemObservation_.input;
    
    for(size_t i = 0; i < jointStates->name.size(); ++i)
    {
      const size_t currentIndex = jointNameIndexMap_[jointStates->name[i]];
      currentState[stateOffset_ + currentIndex] = jointStates->position[i];
      currentInput[inputOffset_ + currentIndex] = jointStates->velocity[i];
    }

    if(mpcMrtPtr_)
    {
      mpcMrtPtr_->setCurrentObservation(currentSystemObservation_);
    }
  }

  void LeggedMpcController::updateBasePose(
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr basePose)
  {
    if(this->get_clock()->now() - lastBasePoseTime_ > maxDurationBetweenMessages_)
    {
      RCLCPP_WARN(this->get_logger(), 
        "Time between two PoseStamped messages was longer than maximum duration!");
    }

    lastBasePoseTime_.seconds = basePose->header.stamp.sec;
    lastBasePoseTime_.nanoseconds = basePose->header.stamp.nanosec;

    auto& currentState = currentSystemObservation_.state;

    currentState(7) = basePose->pose.position.x;
    currentState(8) = basePose->pose.position.y;
    currentState(9) = basePose->pose.position.z;

    
  }

  void LeggedMpcController::updateBaseTwist(
    const geometry_msgs::msg::TwistStamped::ConstSharedPtr baseTwist)
  {

  }

  void LeggedMpcController::updateContactFlags(
    const contact_msgs::msg::Contacts::ConstSharedPtr contactFlags)
  {

  }

  void LeggedMpcController::updateGaitParameters(
    const legged_locomotion_msgs::msg::GaitParameters::ConstSharedPtr gaitParameters)
  {

  }

} // namespace legged_locomotion_mpc_ros2
