// Copyright (c) 2025, Bartłomiej Krajewski
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

/*
 * Authors: Bartłomiej Krajewski (https://github.com/BartlomiejK2)
 * Based on: Farbod Farshidian (https://github.com/leggedrobotics/ocs2)
 */

#include <legged_locomotion_mpc_ros2/visualization/LeggedVisualizer.hpp>

#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <ocs2_ros_interfaces/visualization/VisualizationHelpers.h>

#include <floating_base_model/AccessHelperFunctions.hpp>

#include <legged_locomotion_mpc/locomotion/GaitCommon.hpp>

namespace legged_locomotion_mpc_ros2
{
  using namespace ocs2;
  using namespace pinocchio;
  using namespace floating_base_model;
  using namespace access_helper_functions;
  using namespace legged_locomotion_mpc;
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  LeggedVisualizer::LeggedVisualizer(LeggedVisualizer::Settings visualizationSettings,
    ModelSettings modelSettings, floating_base_model::FloatingBaseModelInfo modelInfo, 
    const Model& robotModel, 
    const PinocchioForwardEndEffectorKinematicsCppAd& forwardKinematics, 
    const PinocchioTorqueApproximationCppAd& torqueApproximator):
      rclcpp::Node("legged_visualizer"), 
      visualizationSettings_(std::move(visualizationSettings)), 
      modelSettings_(std::move(modelSettings)), modelInfo_(std::move(modelInfo)), 
      robotName_(robotModel.name), forwardKinematics_(forwardKinematics), 
      torqueApproximator_(torqueApproximator)
  {
    jointNames_ = robotModel.names;

    // Remove "universe" and "root_joint" joints from joint names
    jointNames_.erase(std::remove(jointNames_.begin(), jointNames_.end(), "universe"), jointNames_.end());
    jointNames_.erase(std::remove(jointNames_.begin(), jointNames_.end(), "root_joint"), jointNames_.end());

    baseTransformBroadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    const std::string topicPrefix = "/" + robotName_ + "/";

    const std::string baseTwistTopic = topicPrefix + "base_twist";
    baseTwistPublisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      baseTwistTopic, 1);
    
    const std::string jointStateTopic = "joint_states";
    jointStatePublisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
      jointStateTopic, 1);
    
    const std::string desiredBasePositionTopic = topicPrefix + "base_desired_positon";
    desiredBasePositionPublisher_ = this->create_publisher<
      visualization_msgs::msg::Marker>(desiredBasePositionTopic, 1);

    std::vector<std::string> endEffectorNames = modelSettings_.endEffectorThreeDofNames;
    endEffectorNames.insert(endEffectorNames.end(), 
      modelSettings_.endEffectorSixDofNames.begin(), 
      modelSettings_.endEffectorSixDofNames.end());

    for(const auto& endEffectorName: endEffectorNames)
    {
      const std::string desiredEndEffectorDataTopic = topicPrefix 
        + endEffectorName + "_desired_position";
      const auto desiredEndEffectorDataPublisher = this->create_publisher<
        visualization_msgs::msg::Marker>(desiredEndEffectorDataTopic, 1);

      desiredEndEffectorDataPublishers_.push_back(desiredEndEffectorDataPublisher);
    }

    const std::string optimizedBasePositionTopic = topicPrefix + "base_optimized_positon";
    optimizedBasePositionPublisher_ = this->create_publisher<
      visualization_msgs::msg::Marker>(optimizedBasePositionTopic, 1);

    for(const auto& endEffectorName: endEffectorNames)
    {
      const std::string optimizedEndEffectorDataTopic = topicPrefix 
        + endEffectorName + "_optimized_position";
      const auto optimizedEndEffectorDataPublisher = this->create_publisher<
        visualization_msgs::msg::Marker>(optimizedEndEffectorDataTopic, 1);

      optimizedEndEffectorDataPublishers_.push_back(optimizedEndEffectorDataPublisher);
    }

    const std::string currentStateTopic = topicPrefix + "current_end_effector_state";
    currentStatePublisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      currentStateTopic, 1);
    
    // Latched, terrain can be published rarely and still reach late subscribers
    const std::string gridMapTopic = topicPrefix + "elevation";
    gridMapPublisher_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
      gridMapTopic, rclcpp::QoS(1).transient_local());
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void LeggedVisualizer::publishObservation(const rclcpp::Time& timeStamp, 
    const SystemObservation& currentObservation)
  {
    const auto& state = currentObservation.state;
    const auto& input = currentObservation.input;

    const contact_flags_t currentFlags = locomotion::modeNumber2ContactFlags(
      currentObservation.mode);

    if(state.size() != modelInfo_.stateDim || input.size() != modelInfo_.inputDim)
    {
      return;
    }

    const vector3_t basePosition = getBasePosition(state, modelInfo_);
    const vector3_t baseEulerAngles = getBaseOrientationZyx(state, modelInfo_);

    const vector3_t baseLinearVelocity = getBaseLinearVelocity(state, modelInfo_);
    const vector3_t baseAngularVelocity = getBaseAngularVelocity(state, modelInfo_);

    const vector_t jointPositions = getJointPositions(state, modelInfo_);

    const vector_t jointVelocities = getJointVelocities(input, modelInfo_);

    const std::vector<vector3_t> endEffectorPositions = forwardKinematics_.getPosition(
      state);

    const size_t endEffectorNumber = modelInfo_.numThreeDofContacts 
      + modelInfo_.numSixDofContacts;

    std::vector<vector3_t> endEffectorForces;
    endEffectorForces.reserve(endEffectorNumber);

    std::vector<vector3_t> endEffectorTorques;
    endEffectorTorques.reserve(modelInfo_.numSixDofContacts);

    for(size_t i = 0; i < modelInfo_.numThreeDofContacts; ++i)
    {
      const vector3_t force = getContactForces(input, i, modelInfo_);
      endEffectorForces.push_back(std::move(force));
    }

    for(size_t i = modelInfo_.numThreeDofContacts; i < endEffectorNumber; ++i)
    {
      const vector3_t force = getContactForces(input, i, modelInfo_);
      endEffectorForces.push_back(std::move(force));

      const vector3_t torque = getContactTorques(input, i, modelInfo_);
      endEffectorTorques.push_back(std::move(torque));
    }

    const vector_t jointTorques = torqueApproximator_.getValue(state, input);

    publishBaseTransform(timeStamp, basePosition, baseEulerAngles);
    publishBaseTwist(timeStamp, baseLinearVelocity, baseAngularVelocity);
    publishJointStates(timeStamp, jointPositions, jointVelocities, jointTorques);
    publishEndEffectorMarkers(timeStamp, currentFlags, endEffectorPositions, 
      endEffectorForces, endEffectorTorques);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void LeggedVisualizer::publishTargetTrajectory(const rclcpp::Time& timeStamp,
    const TargetTrajectories& targetTrajectory)
  {
    const auto& timeTrajectory = targetTrajectory.timeTrajectory;
    const auto& stateTrajectory = targetTrajectory.stateTrajectory;

    if(timeTrajectory.empty() || stateTrajectory.empty())
    {
      return;
    }

    std::vector<size_t> reservedIndexes{0};
    scalar_t lastTime = timeTrajectory.front();

    for(size_t i = 1; i < timeTrajectory.size(); ++i)
    {
      if((timeTrajectory[i] - lastTime) > visualizationSettings_.deltaTime)
      {
        lastTime = timeTrajectory[i];
        reservedIndexes.push_back(i);
      }
    }

    const size_t reservedSize = reservedIndexes.size();

    const size_t endEffectorNumber = modelInfo_.numThreeDofContacts 
      + modelInfo_.numSixDofContacts;

    // Reserve base position messages
    std::vector<geometry_msgs::msg::Point> desiredBasePositionMessages;
    desiredBasePositionMessages.reserve(reservedSize);

    // Reserve end effector messages
    std::vector<std::vector<geometry_msgs::msg::Point>> 
      desiredEndEffectorPositionMessages(endEffectorNumber);

    for(size_t i = 0; i < endEffectorNumber; ++i) 
    {
      desiredEndEffectorPositionMessages[i].reserve(reservedSize);
    }

    for(size_t i = 0; i < reservedSize; ++i) 
    {
      const size_t currentIndex = reservedIndexes[i];
      const auto& state = stateTrajectory[currentIndex];

      // Construct base point message
      const vector3_t basePosition = getBasePosition(state, modelInfo_);
      geometry_msgs::msg::Point basePoint;
      basePoint = getPointMsg(basePosition);

      // Fill message containers
      desiredBasePositionMessages.push_back(std::move(basePoint));

      // Fill end effector messages
      const std::vector<vector3_t> endEffectorPositions = forwardKinematics_.getPosition(
        state);

      for(size_t j = 0; j < endEffectorNumber; ++j) 
      {
        geometry_msgs::msg::Point endEffectorPoint;
        endEffectorPoint = getPointMsg(endEffectorPositions[j]);
        desiredEndEffectorPositionMessages[j].push_back(std::move(endEffectorPoint));
      }
    }

    // Header
    const auto headerMessage = getHeaderMsg(modelSettings_.worldLinkName, timeStamp);

    auto basePathMessage = getLineMsg(std::move(desiredBasePositionMessages), 
      Color::green, visualizationSettings_.trajectoryLineWidth);
    basePathMessage.header = headerMessage;
    basePathMessage.id = 0;

    // Publish
    desiredBasePositionPublisher_->publish(basePathMessage);

    const size_t colorMapSize = visualizationSettings_.endEffectorColorMap.size();

    for(size_t j = 0; j < endEffectorNumber; ++j) 
    {
      auto endEffectorPathMessage = getLineMsg(
        std::move(desiredEndEffectorPositionMessages[j]),
        visualizationSettings_.endEffectorColorMap[j % colorMapSize],
        visualizationSettings_.trajectoryLineWidth);
      endEffectorPathMessage.header = headerMessage;
      endEffectorPathMessage.id = 0;

      desiredEndEffectorDataPublishers_[j]->publish(endEffectorPathMessage);
    }
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void LeggedVisualizer::publishOptimizedTrajectory(const rclcpp::Time& timeStamp, 
    const scalar_array_t& optimizedTimeTrajectory, 
    const vector_array_t& optimizedStateTrajectory)
  {
    if (optimizedTimeTrajectory.empty() || optimizedStateTrajectory.empty()) 
    {
      return;
    }

    const auto& timeTrajectory = optimizedTimeTrajectory;
    const auto& stateTrajectory = optimizedStateTrajectory;

    std::vector<size_t> reservedIndexes{0};
    scalar_t lastTime = timeTrajectory.front();

    for(size_t i = 1; i < timeTrajectory.size(); ++i)
    {
      if((timeTrajectory[i] - lastTime) > visualizationSettings_.deltaTime)
      {
        lastTime = timeTrajectory[i];
        reservedIndexes.push_back(i);
      }
    }

    const size_t reservedSize = reservedIndexes.size();

    const size_t endEffectorNumber = modelInfo_.numThreeDofContacts 
      + modelInfo_.numSixDofContacts;

    // Reserve base position messages
    std::vector<geometry_msgs::msg::Point> optimizedBasePositionMessages;
    optimizedBasePositionMessages.reserve(reservedSize);

    // Reserve end effector messages
    std::vector<std::vector<geometry_msgs::msg::Point>> 
      optimizedEndEffectorPositionMessages(endEffectorNumber);

    for(size_t i = 0; i < endEffectorNumber; ++i) 
    {
      optimizedEndEffectorPositionMessages[i].reserve(reservedSize);
    }

    for(size_t i = 0; i < reservedSize; ++i) 
    {
      const size_t currentIndex = reservedIndexes[i];
      const auto& state = stateTrajectory[currentIndex];

      // Construct base point message
      const vector3_t basePosition = getBasePosition(state, modelInfo_);
      geometry_msgs::msg::Point basePoint;
      basePoint = getPointMsg(basePosition);

      // Fill message containers
      optimizedBasePositionMessages.push_back(std::move(basePoint));

      // Fill end effector messages
      const std::vector<vector3_t> endEffectorPositions = forwardKinematics_.getPosition(
        state);

      for(size_t j = 0; j < endEffectorNumber; ++j) 
      {
        geometry_msgs::msg::Point endEffectorPoint;
        endEffectorPoint = getPointMsg(endEffectorPositions[j]);
        optimizedEndEffectorPositionMessages[j].push_back(std::move(endEffectorPoint));
      }
    }

    // Header
    const auto headerMessage = getHeaderMsg(modelSettings_.worldLinkName, timeStamp);

    auto basePathMessage = getLineMsg(std::move(optimizedBasePositionMessages), 
      Color::green, visualizationSettings_.trajectoryLineWidth);
    basePathMessage.header = headerMessage;
    basePathMessage.id = 0;

    // Publish
    optimizedBasePositionPublisher_->publish(basePathMessage);

    const size_t colorMapSize = visualizationSettings_.endEffectorColorMap.size();

    for(size_t j = 0; j < endEffectorNumber; ++j) 
    {
      auto endEffectorPathMessage = getLineMsg(
        std::move(optimizedEndEffectorPositionMessages[j]),
        visualizationSettings_.endEffectorColorMap[j % colorMapSize],
        visualizationSettings_.trajectoryLineWidth);
      endEffectorPathMessage.header = headerMessage;
      endEffectorPathMessage.id = 0;

      optimizedEndEffectorDataPublishers_[j]->publish(endEffectorPathMessage);
    }
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void LeggedVisualizer::publishJointStates(const rclcpp::Time& timeStamp, 
    const vector_t& jointPositons, const vector_t& jointVelocities, 
    const vector_t& jointTorques)
  {
    // Copy Eigen data directly into message
    sensor_msgs::msg::JointState jointState;
    jointState.header.stamp = timeStamp;
    jointState.name = jointNames_;
    jointState.position.assign(jointPositons.data(),
      jointPositons.data() + jointPositons.size());
    jointState.velocity.assign(jointVelocities.data(),
      jointVelocities.data() + jointVelocities.size());
    jointState.effort.assign(jointTorques.data(),
      jointTorques.data() + jointTorques.size());

    jointStatePublisher_->publish(jointState);
  }
  
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void LeggedVisualizer::publishBaseTransform(const rclcpp::Time& timeStamp, 
    std::string frameName, const vector3_t& basePositon, const vector3_t& baseEulerAngles)
  {
    geometry_msgs::msg::TransformStamped baseTransform;

    baseTransform.header.stamp = timeStamp;
    baseTransform.header.frame_id = modelSettings_.worldLinkName;
    baseTransform.child_frame_id = frameName;
    baseTransform.transform.translation.x = basePositon.x();
    baseTransform.transform.translation.y = basePositon.y();
    baseTransform.transform.translation.z = basePositon.z();

    const auto baseQuaterion = getQuaternionFromEulerAnglesZyx(baseEulerAngles);

    baseTransform.transform.rotation.x = baseQuaterion.x();
    baseTransform.transform.rotation.y = baseQuaterion.y();
    baseTransform.transform.rotation.z = baseQuaterion.z();
    baseTransform.transform.rotation.w = baseQuaterion.w();

    baseTransformBroadcaster_->sendTransform(baseTransform);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void LeggedVisualizer::publishBaseTransform(const rclcpp::Time& timeStamp, 
    const vector3_t& basePositon, const vector3_t& baseEulerAngles)
  {
    publishBaseTransform(timeStamp, modelSettings_.baseLinkName, 
      basePositon, baseEulerAngles);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void LeggedVisualizer::publishBaseTwist(const rclcpp::Time& timeStamp, 
    const vector3_t& baseLinearVelocity, const vector3_t& baseAngularVelocity)
  {
    geometry_msgs::msg::TwistStamped baseTwist;

    baseTwist.header.stamp = timeStamp;
    baseTwist.header.frame_id = modelSettings_.baseLinkName;

    baseTwist.twist.linear.x = baseLinearVelocity.x();
    baseTwist.twist.linear.y = baseLinearVelocity.y();
    baseTwist.twist.linear.z = baseLinearVelocity.z();

    baseTwist.twist.angular.x = baseAngularVelocity.x();
    baseTwist.twist.angular.y = baseAngularVelocity.y();
    baseTwist.twist.angular.z = baseAngularVelocity.z();

    baseTwistPublisher_->publish(baseTwist);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void LeggedVisualizer::publishEndEffectorMarkers(const rclcpp::Time& timeStamp, 
    const contact_flags_t& contactFlags,
    const std::vector<vector3_t>& endEffectorPositions,
    const std::vector<vector3_t>& endEffectorForces, 
    const std::vector<vector3_t>& endEffectorTorques)
  {
    const size_t endEffectorNumber = modelInfo_.numThreeDofContacts 
      + modelInfo_.numSixDofContacts;

    const size_t numMarkers = 2 * modelInfo_.numThreeDofContacts 
      + 3 * modelInfo_.numSixDofContacts + 2;

    visualization_msgs::msg::MarkerArray markerArray;
    markerArray.markers.reserve(numMarkers);

    const size_t colorMapSize = visualizationSettings_.endEffectorColorMap.size();

    // Markers for three DOF end effectors (not const, so they are moved into array)
    for(size_t i = 0; i < modelInfo_.numThreeDofContacts; ++i)
    {
      auto positionMarker = getFootMarker(endEffectorPositions[i], contactFlags[i],
        visualizationSettings_.endEffectorColorMap[i % colorMapSize],
        visualizationSettings_.endEffectorMarkerDiameter,
        visualizationSettings_.endEffectorAlphaWhenLifted);

      auto forceMarker = getForceMarker(endEffectorForces[i],
        endEffectorPositions[i], contactFlags[i], Color::green, 
        visualizationSettings_.forceScale);

      markerArray.markers.emplace_back(std::move(positionMarker));
      markerArray.markers.emplace_back(std::move(forceMarker));
    }

    // Markers for six DOF end effectors
    for(size_t i = modelInfo_.numThreeDofContacts; i < endEffectorNumber; ++i)
    {
      auto positionMarker = getFootMarker(endEffectorPositions[i], contactFlags[i],
        visualizationSettings_.endEffectorColorMap[i % colorMapSize],
        visualizationSettings_.endEffectorMarkerDiameter,
        visualizationSettings_.endEffectorAlphaWhenLifted);

      auto forceMarker = getForceMarker(endEffectorForces[i],
        endEffectorPositions[i], contactFlags[i], Color::green,
        visualizationSettings_.forceScale);

      auto torqueMarker = getForceMarker(
        endEffectorTorques[i - modelInfo_.numThreeDofContacts], 
        endEffectorPositions[i], contactFlags[i], Color::red, 
        visualizationSettings_.torqueScale);

      markerArray.markers.emplace_back(std::move(positionMarker));
      markerArray.markers.emplace_back(std::move(forceMarker));
      markerArray.markers.emplace_back(std::move(torqueMarker));
    }

    // Get std::vector<bool> from contact_flags_t
    std::vector<bool> currentFlagsAsBool(endEffectorNumber);
    for(size_t i = 0; i < endEffectorNumber; ++i)
    {
      currentFlagsAsBool[i] = contactFlags[i] > 0;
    }

    // Center of pressure
    auto centerOfPressureMarker = getCenterOfPressureMarker(
      endEffectorForces.begin(), endEffectorForces.end(), endEffectorPositions.begin(),
      currentFlagsAsBool.begin(), Color::green, 
      visualizationSettings_.centerOfPressureMarkerDiameter);

    // Support polygon
    auto supportPolygonMarker = getSupportPolygonMarker(
      endEffectorPositions.begin(), endEffectorPositions.end(), 
      currentFlagsAsBool.begin(), Color::black, 
      visualizationSettings_.supportPolygonLineWidth);

    markerArray.markers.emplace_back(std::move(centerOfPressureMarker));
    markerArray.markers.emplace_back(std::move(supportPolygonMarker));

    // Give markers an id and a frame
    const auto headerMessage = getHeaderMsg(modelSettings_.worldLinkName, timeStamp);
    assignHeader(markerArray.markers.begin(), markerArray.markers.end(),
      headerMessage);
    assignIncreasingId(markerArray.markers.begin(), markerArray.markers.end());

    // Publish cartesian markers (minus the CoM Pose)
    currentStatePublisher_->publish(markerArray);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void LeggedVisualizer::publishTerrainModel(const terrain_model::TerrainModel& terrainModel)
  {
    auto gridMapMessage =
    grid_map::GridMapRosConverter::toMessage(terrainModel.getGridMapTerrain(), 
      {terrain_model::elevationLayerName});

    gridMapPublisher_->publish(std::move(gridMapMessage));
  }
} // namespace legged_locomotion_mpc
