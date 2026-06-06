
#include <gtest/gtest.h>
#include <legged_locomotion_mpc/trajectory_planners/BaseTrajectoryPlanner.hpp>
#include <legged_locomotion_mpc/common/ModelSettings.hpp>

#include <terrain_model/core/TerrainPlane.hpp>
#include <terrain_model/planar_model/PlanarTerrainModel.hpp>

#include <floating_base_model/AccessHelperFunctions.hpp>
#include <floating_base_model/FactoryFunctions.hpp>

#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <floating_base_model/QuaterionEulerTransforms.hpp>

#include <pinocchio/spatial/explog.hpp>

#include "../include/definitions.hpp"

using namespace legged_locomotion_mpc;
using namespace legged_locomotion_mpc::planners;
using namespace ocs2;
using namespace terrain_model;
using namespace floating_base_model;

const scalar_t tol = 1e-9;

TEST(BaseTrajectoryPlannerTest, translationOnFlatTerrain) 
{
  const scalar_t slopeY = 0.0;
  const scalar_t slopeX = 0.0;
  const vector3_t terrainEulerZyx{0.0, slopeY, slopeX};
  const matrix3_t terrainRotation = getRotationMatrixFromZyxEulerAngles(terrainEulerZyx); 

  TerrainPlane slopyTerrain(vector3_t::Random(), terrainRotation.transpose());

  PlanarTerrainModel terrainModel(slopyTerrain);

  FloatingBaseModelInfo modelInfo;
  modelInfo.actuatedDofNum = 12;
  modelInfo.generalizedCoordinatesNum = 12 + 6;
  modelInfo.inputDim = 12 + 12;
  modelInfo.stateDim = 12 + 12;
  modelInfo.numSixDofContacts = 0;
  modelInfo.numThreeDofContacts = 4;

  BaseTrajectoryPlanner::StaticSettings staticSettings;
  staticSettings.deltaTime = 0.1;
  scalar_t initialBaseHeight = 2.5;
  staticSettings.minimumBaseHeight = 0.1;
  staticSettings.maximumBaseHeight = 5.0;
  staticSettings.nominalBaseWidthHeading = 0.2;
  staticSettings.nominalBaseWidthLateral = 0.2;
  staticSettings.maximumBaseHeadingVelocity = 1.0;
  staticSettings.maximumBaseLateralVelocity = 1.0;
  staticSettings.maximumBaseVerticalVelocity = 1.0;
  staticSettings.maximumYawRate = 1.0;

  BaseTrajectoryPlanner planner(modelInfo, staticSettings);
  planner.updateTerrain(terrainModel);

  const scalar_t initTime = 0.5;
  const scalar_t finalTime = 1.5;

  BaseTrajectoryPlanner::BaseReferenceCommand command;
  command.baseHeadingVelocity = std::rand() / scalar_t(RAND_MAX);
  command.baseLateralVelocity = std::rand() / scalar_t(RAND_MAX);
  command.baseVerticalVelocity = std::rand() / scalar_t(RAND_MAX);
  command.yawRate = 0.0;

  const vector3_t initPosition = vector3_t::Random();

  const vector3_t planeLocalEulerZyx = vector3_t{std::rand() / scalar_t(RAND_MAX), 0.0, 
    0.0};
  matrix3_t initRotationOnPlane = getRotationMatrixFromZyxEulerAngles(planeLocalEulerZyx);

  const vector3_t headingVector = initRotationOnPlane.col(0);
  const vector3_t xAxis = slopyTerrain.projectVectorInWorldOntoPlaneAlongGravity(headingVector).normalized();
  const vector3_t zAxis = slopyTerrain.getSurfaceNormalInWorld();
  const vector3_t yAxis = zAxis.cross(xAxis);

  matrix3_t initRotationMatrix;
  initRotationMatrix.col(0) = xAxis;
  initRotationMatrix.col(1) = yAxis;
  initRotationMatrix.col(2) = zAxis;

  const vector3_t initEulerZyx = quaterion_euler_transforms::getEulerAnglesFromRotationMatrix(
    initRotationMatrix);

  const vector2_t initPositionXY{initPosition.x(), initPosition.y()};

  vector3_t initialBasePosition(initPosition.x(), initPosition.y(),
    terrainModel.getSmoothedPositon(initPositionXY).z());
  
  initialBasePosition.z() += initialBaseHeight / slopyTerrain.getSurfaceNormalInWorld().z();
  
  SystemObservation initialObservation;
  initialObservation.state = vector_t::Zero(modelInfo.stateDim);
  initialObservation.input = vector_t::Zero(modelInfo.inputDim);
  initialObservation.state.block<3,1>(6, 0) = initialBasePosition;
  initialObservation.state.block<3,1>(9, 0) = initEulerZyx;

  TargetTrajectories trajectories;

  planner.updateTargetTrajectory(initTime, finalTime, command, 
    initialObservation, trajectories);

  const size_t size = (finalTime - initTime) / staticSettings.deltaTime + 1;

  vector3_t basePositionTrue = initialBasePosition;
  vector3_t baseOrientationZyxTrue = initEulerZyx;
  const vector3_t baseLinearVelocityTrue = vector3_t(command.baseHeadingVelocity, 
    command.baseLateralVelocity, command.baseVerticalVelocity);
  const vector3_t baseAngularVelocityTrue = vector3_t(0.0, 0.0, command.yawRate);

  for (size_t i = 0; i < size; ++i) 
  {
    const scalar_t currentTime = trajectories.timeTrajectory[i];
    const vector_t& currentState = trajectories.stateTrajectory[i];

    using namespace access_helper_functions;
    const vector3_t& basePosition = getBasePosition(currentState, modelInfo);
    const vector3_t& baseOrientationZyx = getBaseOrientationZyx(currentState, modelInfo);
    const vector3_t& baseLinearVelocity = getBaseLinearVelocity(currentState, modelInfo);
    const vector3_t& baseAngularVelocity = getBaseAngularVelocity(currentState, modelInfo);

    const matrix3_t currentRotation = getRotationMatrixFromZyxEulerAngles(
      baseOrientationZyxTrue);

    pinocchio::SE3 currentTransform(currentRotation, basePositionTrue);

    const pinocchio::Motion twistDelta(baseLinearVelocityTrue * staticSettings.deltaTime, 
      baseAngularVelocityTrue * staticSettings.deltaTime);

    pinocchio::SE3 delta = pinocchio::exp6(
        twistDelta);

    const pinocchio::SE3 newTransform = currentTransform * delta;

    const scalar_t currentTimeTrue = initTime + i * staticSettings.deltaTime;

    EXPECT_TRUE(std::abs(currentTime - currentTimeTrue) < tol);
    EXPECT_TRUE((basePosition - basePositionTrue).norm() < tol);
    EXPECT_TRUE((baseOrientationZyx - baseOrientationZyxTrue).norm() < tol);

    // Linear and angular velocities are in BASE frame!
    EXPECT_TRUE((baseLinearVelocity - baseLinearVelocityTrue).norm() < tol);
    EXPECT_TRUE((baseAngularVelocity - baseAngularVelocityTrue).norm() < tol);

    basePositionTrue = newTransform.translation();
    
    const matrix3_t newRotation = newTransform.rotation(); 
    baseOrientationZyxTrue = quaterion_euler_transforms::getEulerAnglesFromRotationMatrix(
      newRotation);
  }
}

TEST(BaseTrajectoryPlannerTest, rotationOnFlatTerrain) 
{
  const scalar_t slopeY = 0.0;
  const scalar_t slopeX = 0.0;
  const vector3_t terrainEulerZyx{0.0, slopeY, slopeX};
  const matrix3_t terrainRotation = getRotationMatrixFromZyxEulerAngles(terrainEulerZyx); 

  TerrainPlane slopyTerrain(vector3_t::Random(), terrainRotation.transpose());

  PlanarTerrainModel terrainModel(slopyTerrain);

  FloatingBaseModelInfo modelInfo;
  modelInfo.actuatedDofNum = 12;
  modelInfo.generalizedCoordinatesNum = 12 + 6;
  modelInfo.inputDim = 12 + 12;
  modelInfo.stateDim = 12 + 12;
  modelInfo.numSixDofContacts = 0;
  modelInfo.numThreeDofContacts = 4;

  BaseTrajectoryPlanner::StaticSettings staticSettings;
  staticSettings.deltaTime = 0.1;
  scalar_t initialBaseHeight = 2.5;
  staticSettings.minimumBaseHeight = 0.1;
  staticSettings.maximumBaseHeight = 5.0;
  staticSettings.nominalBaseWidthHeading = 0.2;
  staticSettings.nominalBaseWidthLateral = 0.2;
  staticSettings.maximumBaseHeadingVelocity = 1.0;
  staticSettings.maximumBaseLateralVelocity = 1.0;
  staticSettings.maximumBaseVerticalVelocity = 1.0;
  staticSettings.maximumYawRate = 1.0;

  BaseTrajectoryPlanner planner(modelInfo, staticSettings);
  planner.updateTerrain(terrainModel);

  const scalar_t initTime = 0.5;
  const scalar_t finalTime = 1.5;

  BaseTrajectoryPlanner::BaseReferenceCommand command;
  command.baseHeadingVelocity = 0.0;
  command.baseLateralVelocity = 0.0;
  command.baseVerticalVelocity = 0.0;
  command.yawRate = std::rand() / scalar_t(RAND_MAX);

  const vector3_t initPosition = vector3_t::Random();

  const vector3_t planeLocalEulerZyx = vector3_t{std::rand() / scalar_t(RAND_MAX), 0.0, 
    0.0};
  matrix3_t initRotationOnPlane = getRotationMatrixFromZyxEulerAngles(planeLocalEulerZyx);

  const vector3_t headingVector = initRotationOnPlane.col(0);
  const vector3_t xAxis = slopyTerrain.projectVectorInWorldOntoPlaneAlongGravity(headingVector).normalized();
  const vector3_t zAxis = slopyTerrain.getSurfaceNormalInWorld();
  const vector3_t yAxis = zAxis.cross(xAxis);

  matrix3_t initRotationMatrix;
  initRotationMatrix.col(0) = xAxis;
  initRotationMatrix.col(1) = yAxis;
  initRotationMatrix.col(2) = zAxis;

  const vector3_t initEulerZyx = quaterion_euler_transforms::getEulerAnglesFromRotationMatrix(
    initRotationMatrix);

  const vector2_t initPositionXY{initPosition.x(), initPosition.y()};

  vector3_t initialBasePosition(initPosition.x(), initPosition.y(),
    terrainModel.getSmoothedPositon(initPositionXY).z());
  
  initialBasePosition.z() += initialBaseHeight / slopyTerrain.getSurfaceNormalInWorld().z();
  
  SystemObservation initialObservation;
  initialObservation.state = vector_t::Zero(modelInfo.stateDim);
  initialObservation.input = vector_t::Zero(modelInfo.inputDim);
  initialObservation.state.block<3,1>(6, 0) = initialBasePosition;
  initialObservation.state.block<3,1>(9, 0) = initEulerZyx;

  TargetTrajectories trajectories;

  planner.updateTargetTrajectory(initTime, finalTime, command, 
    initialObservation, trajectories);

  const size_t size = (finalTime - initTime) / staticSettings.deltaTime + 1;

  vector3_t basePositionTrue = initialBasePosition;
  vector3_t baseOrientationZyxTrue = initEulerZyx;
  const vector3_t baseLinearVelocityTrue = vector3_t(command.baseHeadingVelocity, 
    command.baseLateralVelocity, command.baseVerticalVelocity);
  const vector3_t baseAngularVelocityTrue = vector3_t(0.0, 0.0, command.yawRate);

  for (size_t i = 0; i < size; ++i) 
  {
    const scalar_t currentTime = trajectories.timeTrajectory[i];
    const vector_t& currentState = trajectories.stateTrajectory[i];

    using namespace access_helper_functions;
    const vector3_t& basePosition = getBasePosition(currentState, modelInfo);
    const vector3_t& baseOrientationZyx = getBaseOrientationZyx(currentState, modelInfo);
    const vector3_t& baseLinearVelocity = getBaseLinearVelocity(currentState, modelInfo);
    const vector3_t& baseAngularVelocity = getBaseAngularVelocity(currentState, modelInfo);

    const matrix3_t currentRotation = getRotationMatrixFromZyxEulerAngles(
      baseOrientationZyxTrue);

    pinocchio::SE3 currentTransform(currentRotation, basePositionTrue);

    const pinocchio::Motion twistDelta(baseLinearVelocityTrue * staticSettings.deltaTime, 
      baseAngularVelocityTrue * staticSettings.deltaTime);

    pinocchio::SE3 delta = pinocchio::exp6(
        twistDelta);

    const pinocchio::SE3 newTransform = currentTransform * delta;

    const scalar_t currentTimeTrue = initTime + i * staticSettings.deltaTime;

    EXPECT_TRUE(std::abs(currentTime - currentTimeTrue) < tol);
    EXPECT_TRUE((basePosition - basePositionTrue).norm() < tol);
    EXPECT_TRUE((baseOrientationZyx - baseOrientationZyxTrue).norm() < tol);

    // Linear and angular velocities are in BASE frame!
    EXPECT_TRUE((baseLinearVelocity - baseLinearVelocityTrue).norm() < tol);
    EXPECT_TRUE((baseAngularVelocity - baseAngularVelocityTrue).norm() < tol);

    basePositionTrue = newTransform.translation();
    
    const matrix3_t newRotation = newTransform.rotation(); 
    baseOrientationZyxTrue = quaterion_euler_transforms::getEulerAnglesFromRotationMatrix(
      newRotation);
  }
}

TEST(BaseTrajectoryPlannerTest, translationAndrotationOnSlopyTerrain) 
{
  const scalar_t slopeY = std::rand() / scalar_t(RAND_MAX);
  const scalar_t slopeX = std::rand() / scalar_t(RAND_MAX);
  const vector3_t terrainEulerZyx{0.0, slopeY, slopeX};
  const matrix3_t terrainRotation = getRotationMatrixFromZyxEulerAngles(terrainEulerZyx); 

  TerrainPlane slopyTerrain(vector3_t::Random(), terrainRotation.transpose());

  PlanarTerrainModel terrainModel(slopyTerrain);

  FloatingBaseModelInfo modelInfo;
  modelInfo.actuatedDofNum = 12;
  modelInfo.generalizedCoordinatesNum = 12 + 6;
  modelInfo.inputDim = 12 + 12;
  modelInfo.stateDim = 12 + 12;
  modelInfo.numSixDofContacts = 0;
  modelInfo.numThreeDofContacts = 4;

  BaseTrajectoryPlanner::StaticSettings staticSettings;
  staticSettings.deltaTime = 0.1;
  scalar_t initialBaseHeight = 2.5;
  staticSettings.minimumBaseHeight = 0.1;
  staticSettings.maximumBaseHeight = 5.0;
  staticSettings.nominalBaseWidthHeading = 0.2;
  staticSettings.nominalBaseWidthLateral = 0.2;
  staticSettings.maximumBaseHeadingVelocity = 1.0;
  staticSettings.maximumBaseLateralVelocity = 1.0;
  staticSettings.maximumBaseVerticalVelocity = 1.0;
  staticSettings.maximumYawRate = 1.0;

  BaseTrajectoryPlanner planner(modelInfo, staticSettings);
  planner.updateTerrain(terrainModel);

  const scalar_t initTime = 0.5;
  const scalar_t finalTime = 1.5;

  BaseTrajectoryPlanner::BaseReferenceCommand command;
  command.baseHeadingVelocity = std::rand() / scalar_t(RAND_MAX);
  command.baseLateralVelocity = std::rand() / scalar_t(RAND_MAX);
  command.baseVerticalVelocity = std::rand() / scalar_t(RAND_MAX);
  command.yawRate = std::rand() / scalar_t(RAND_MAX);

  const vector3_t initPosition = vector3_t::Random();

  const vector3_t planeLocalEulerZyx = vector3_t{std::rand() / scalar_t(RAND_MAX), 0.0, 
    0.0};
  matrix3_t initRotationOnPlane = getRotationMatrixFromZyxEulerAngles(planeLocalEulerZyx);

  const vector3_t headingVector = initRotationOnPlane.col(0);
  const vector3_t xAxis = slopyTerrain.projectVectorInWorldOntoPlaneAlongGravity(headingVector).normalized();
  const vector3_t zAxis = slopyTerrain.getSurfaceNormalInWorld();
  const vector3_t yAxis = zAxis.cross(xAxis);

  matrix3_t initRotationMatrix;
  initRotationMatrix.col(0) = xAxis;
  initRotationMatrix.col(1) = yAxis;
  initRotationMatrix.col(2) = zAxis;

  const vector3_t initEulerZyx = quaterion_euler_transforms::getEulerAnglesFromRotationMatrix(
    initRotationMatrix);

  const vector2_t initPositionXY{initPosition.x(), initPosition.y()};

  vector3_t initialBasePosition(initPosition.x(), initPosition.y(),
    terrainModel.getSmoothedPositon(initPositionXY).z());
  
  initialBasePosition.z() += initialBaseHeight / slopyTerrain.getSurfaceNormalInWorld().z();
  
  SystemObservation initialObservation;
  initialObservation.state = vector_t::Zero(modelInfo.stateDim);
  initialObservation.input = vector_t::Zero(modelInfo.inputDim);
  initialObservation.state.block<3,1>(6, 0) = initialBasePosition;
  initialObservation.state.block<3,1>(9, 0) = initEulerZyx;

  TargetTrajectories trajectories;

  planner.updateTargetTrajectory(initTime, finalTime, command, 
    initialObservation, trajectories);

  const size_t size = (finalTime - initTime) / staticSettings.deltaTime + 1;

  vector3_t basePositionTrue = initialBasePosition;
  vector3_t baseOrientationZyxTrue = initEulerZyx;
  const vector3_t baseLinearVelocityTrue = vector3_t(command.baseHeadingVelocity, 
    command.baseLateralVelocity, command.baseVerticalVelocity);
  const vector3_t baseAngularVelocityTrue = vector3_t(0.0, 0.0, command.yawRate);

  for (size_t i = 0; i < size; ++i) 
  {
    const scalar_t currentTime = trajectories.timeTrajectory[i];
    const vector_t& currentState = trajectories.stateTrajectory[i];

    using namespace access_helper_functions;
    const vector3_t& basePosition = getBasePosition(currentState, modelInfo);
    const vector3_t& baseOrientationZyx = getBaseOrientationZyx(currentState, modelInfo);
    const vector3_t& baseLinearVelocity = getBaseLinearVelocity(currentState, modelInfo);
    const vector3_t& baseAngularVelocity = getBaseAngularVelocity(currentState, modelInfo);

    const matrix3_t currentRotation = getRotationMatrixFromZyxEulerAngles(
      baseOrientationZyxTrue);

    pinocchio::SE3 currentTransform(currentRotation, basePositionTrue);

    const pinocchio::Motion twistDelta(baseLinearVelocityTrue * staticSettings.deltaTime, 
      baseAngularVelocityTrue * staticSettings.deltaTime);

    pinocchio::SE3 delta = pinocchio::exp6(
        twistDelta);

    const pinocchio::SE3 newTransform = currentTransform * delta;

    const scalar_t currentTimeTrue = initTime + i * staticSettings.deltaTime;

    EXPECT_TRUE(std::abs(currentTime - currentTimeTrue) < tol);
    EXPECT_TRUE((basePosition - basePositionTrue).norm() < tol);
    EXPECT_TRUE((baseOrientationZyx - baseOrientationZyxTrue).norm() < tol);

    // Linear and angular velocities are in BASE frame!
    EXPECT_TRUE((baseLinearVelocity - baseLinearVelocityTrue).norm() < tol);
    EXPECT_TRUE((baseAngularVelocity - baseAngularVelocityTrue).norm() < tol);

    basePositionTrue = newTransform.translation();
    
    const matrix3_t newRotation = newTransform.rotation(); 
    baseOrientationZyxTrue = quaterion_euler_transforms::getEulerAnglesFromRotationMatrix(
      newRotation);
  }
}

TEST(BaseTrajectoryPlannerTest, loader) 
{
  ModelSettings modelSettings;
  modelSettings.baseLinkName =  baseLink;
  modelSettings.endEffectorThreeDofNames = meldog3DofContactNames;
  modelSettings.endEffectorSixDofNames = meldog6DofContactNames;
  modelSettings.hipFrameNames = meldogHipNames;

  std::string urdfPathName = meldogWithBaseLinkUrdfFile;

  PinocchioInterface interface = createPinocchioInterfaceFromUrdfFile(urdfPathName, baseLink);
  const FloatingBaseModelInfo modelInfo = createFloatingBaseModelInfo(interface, meldog3DofContactNames, meldog6DofContactNames);
  
  const std::string baseFilePath = meldogConfigFolder + "base_settings.info";

  const auto baseSettings = loadBasePlannerStaticSettings(baseFilePath);


  EXPECT_TRUE(baseSettings.minimumBaseHeight            == 0.05);
  EXPECT_TRUE(baseSettings.maximumBaseHeight            == 0.2);
  EXPECT_TRUE(baseSettings.nominalBaseWidthLateral      == 0.2);
  EXPECT_TRUE(baseSettings.nominalBaseWidthHeading      == 0.3);
  EXPECT_TRUE(baseSettings.maximumBaseHeadingVelocity   == 0.4);
  EXPECT_TRUE(baseSettings.maximumBaseLateralVelocity   == 0.6);
  EXPECT_TRUE(baseSettings.maximumBaseVerticalVelocity  == 0.7);
  EXPECT_TRUE(baseSettings.maximumYawRate               == 0.8);
}