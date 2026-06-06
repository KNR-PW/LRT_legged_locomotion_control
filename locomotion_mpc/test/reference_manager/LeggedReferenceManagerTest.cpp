#include <gtest/gtest.h>

#include <legged_locomotion_mpc/reference_manager/LeggedReferenceManager.hpp>


#include <floating_base_model/FactoryFunctions.hpp>

#include <terrain_model/core/TerrainPlane.hpp>
#include <terrain_model/planar_model/PlanarTerrainModel.hpp>

#include "../test/include/definitions.hpp"

using namespace legged_locomotion_mpc;
using namespace legged_locomotion_mpc::locomotion;
using namespace legged_locomotion_mpc::planners;
using namespace floating_base_model;
using namespace ocs2;
using namespace terrain_model;
using namespace multi_end_effector_kinematics;

const double eps = 1e-9;
const size_t TEST_NUM = 100;

// Not round because std::lower_bound is sensitive for time points of mode change
const size_t ITERATIONS = 50;

TEST(LeggedReferenceManagerTest, getContactFlags) 
{
  const scalar_t defTime = 0.0;
  const scalar_t initTime = 0.0;
  const scalar_t finalTime = 2.1;
  const scalar_t deltaTime = 0.01;

  /* STANDING TROT */
  scalar_t currentPhase = 3.5 / 7.0;

  GaitStaticParameters staticParams;
  staticParams.endEffectorNumber = 4;
  staticParams.initPhase = currentPhase;
  
  GaitDynamicParameters dynamicParams;
  dynamicParams.steppingFrequency = 1.0 / 0.7;
  dynamicParams.swingRatio = 3.0 / 7.0;
  
  dynamicParams.phaseOffsets = {-currentPhase , -currentPhase , 0};

  const scalar_t slope = 0.0;
  const vector3_t terrainEulerZyx{0.0, slope, 0.0};
  const matrix3_t terrainRotation = getRotationMatrixFromZyxEulerAngles(terrainEulerZyx); 

  TerrainPlane slopyTerrain(vector3_t::Zero(), terrainRotation.transpose());

  PlanarTerrainModel terrainModel(slopyTerrain);

  BaseTrajectoryPlanner::StaticSettings staticSettings;
  staticSettings.deltaTime = deltaTime;
  scalar_t initialBaseHeight = 0.25 * std::sqrt(2.0);
  staticSettings.minimumBaseHeight = 0.1;
  staticSettings.maximumBaseHeight = 5.0;
  staticSettings.nominalBaseWidthHeading = 0.2;
  staticSettings.nominalBaseWidthLateral = 0.2;
  staticSettings.maximumBaseHeadingVelocity = 1.0;
  staticSettings.maximumBaseLateralVelocity = 1.0;
  staticSettings.maximumBaseVerticalVelocity = 1.0;
  staticSettings.maximumYawRate = 1.0;

  std::string urdfPathName = meldogWithBaseLinkUrdfFile;

  std::string solverName = "NewtonRaphson";

  KinematicsModelSettings modelSettings;
  modelSettings.baseLinkName = baseLink;
  modelSettings.threeDofEndEffectorNames = meldog3DofContactNames;

  InverseSolverSettings solverSettings;
  // solverSettings.stepCoefficient = 0.1;
  solverSettings.tolerance = 1e-3;
  MultiEndEffectorKinematics kinematicsSolver(urdfPathName, modelSettings, 
    solverSettings, solverName);

  ocs2::PinocchioInterface interface = createPinocchioInterfaceFromUrdfFile(urdfPathName, baseLink);
  const FloatingBaseModelInfo modelInfo = createFloatingBaseModelInfo(interface, meldog3DofContactNames, meldog6DofContactNames);
  
  BaseTrajectoryPlanner::BaseReferenceCommand command;
  command.baseHeadingVelocity = std::rand() / scalar_t(RAND_MAX);
  command.baseLateralVelocity = std::rand() / scalar_t(RAND_MAX);
  command.baseVerticalVelocity = 0.0;
  command.yawRate = std::rand() / scalar_t(RAND_MAX);
  
  std::string modelName = "meldogForwardKinematics";
  PinocchioForwardEndEffectorKinematicsCppAd forwardKinematics(interface, 
    modelInfo, modelName);

  SwingTrajectoryPlanner::StaticSettings swingStaticSettings;
  SwingTrajectoryPlanner::DynamicSettings swingDynamicSettings;
  swingDynamicSettings.phases.resize(4);
  swingDynamicSettings.swingHeights.resize(4);
  swingDynamicSettings.tangentialProgresses.resize(4);
  swingDynamicSettings.tangentialVelocityFactors.resize(4);
  for(size_t i = 0; i < 4; ++i)
  {
    swingDynamicSettings.phases[i] = 0.5;
    swingDynamicSettings.swingHeights[i] = 0.1;
    swingDynamicSettings.tangentialProgresses[i] = 0.5;
    swingDynamicSettings.tangentialVelocityFactors[i] = 2.0;
  }

  GaitPlanner gaitPlanner(staticParams, dynamicParams, defTime);

  GaitPlanner trueGaitPlanner = gaitPlanner;

  BaseTrajectoryPlanner basePlanner(modelInfo, staticSettings);

  ModelSettings leggedModelSettings;
  leggedModelSettings.baseLinkName =  baseLink;
  leggedModelSettings.endEffectorThreeDofNames = meldog3DofContactNames;
  leggedModelSettings.endEffectorSixDofNames = meldog6DofContactNames;
  leggedModelSettings.hipFrameNames = meldogHipNames;

  OverExtensionPenalty::Settings penaltySettings;
  penaltySettings.nominalLegExtension = 0.5;
  penaltySettings.legOverExtensionWeight = 1.0;

  const std::string penaltyName = "over_extension_penalty";

  OverExtensionPenalty penalty(interface, leggedModelSettings, penaltySettings, modelInfo, 
    penaltyName);

  SwingTrajectoryPlanner swingPlanner(modelInfo, swingStaticSettings, 
    swingDynamicSettings, forwardKinematics, penalty);

  JointTrajectoryPlanner jointPlanner(modelInfo, std::move(kinematicsSolver));

  PinocchioWeightCompensator compensator(interface, modelInfo);
  ContactForceWrenchTrajectoryPlanner forcePlanner(compensator);
  
  LeggedReferenceManager::Settings managerSettings;

  const auto modeSchedule = gaitPlanner.getModeSchedule(initTime, finalTime);
  contact_flags_t contactFlag = modeSchedule.modeAtTime(initTime);

  LeggedReferenceManager manager(modelInfo, 
    managerSettings, std::move(gaitPlanner), std::move(swingPlanner), std::move(basePlanner), 
    std::move(jointPlanner), std::move(forcePlanner));

  const vector3_t initPosition = vector3_t{0.0, 0.0, 0.0};

  const vector3_t planeLocalEulerZyx = vector3_t{0.0, 0.0, 0.0};

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

  auto& initialState = initialObservation.state;

  access_helper_functions::getJointPositions(initialState, modelInfo) << 0, -0.785398163, 1.570796326, 0, -0.785398163, 1.570796326, 0, -0.785398163, 1.570796326, 0, -0.785398163, 1.570796326;

  std::unique_ptr<TerrainModel> terrainModelPtr = 
    std::make_unique<PlanarTerrainModel>(slopyTerrain);
  
  manager.initialize(initTime, finalTime, initialObservation, contactFlag, 
    dynamicParams, swingDynamicSettings, std::move(terrainModelPtr));

  std::vector<scalar_t> goodTimings = {0.0, 0.3, 0.35, 0.65, 0.7, 1, 1.05, 1.35, 1.4, 1.7, 1.75, 2.05, 2.1, 2.4};
  std::vector<size_t> goodSequence = {15, 9, 15, 6, 15, 9, 15, 6, 15, 9, 15, 6, 15, 9, 15};
  const auto mySequence = manager.getModeSchedule().modeSequence;

  EXPECT_TRUE(mySequence == goodSequence);
  for(size_t i = 0; i < goodTimings.size(); ++i)
  {
    const scalar_t time = goodTimings[i];
    const auto mode = contactFlags2ModeNumber(manager.getContactFlags(time));
    const auto trueMode = goodSequence[i];
    const auto gaitMode = contactFlags2ModeNumber(trueGaitPlanner.getContactFlagsAtTime(time));
    EXPECT_TRUE(mode == trueMode);
    EXPECT_TRUE(gaitMode == trueMode);
  }
}

TEST(LeggedReferenceManagerTest, getTerrainModel) 
{
  const scalar_t defTime = 0.0;
  const scalar_t initTime = 0.0;
  const scalar_t finalTime = 2.1;
  const scalar_t deltaTime = 0.05;

  /* STANDING TROT */
  scalar_t currentPhase = 3.5 / 7.0;

  GaitStaticParameters staticParams;
  staticParams.endEffectorNumber = 4;
  staticParams.initPhase = currentPhase;
  
  GaitDynamicParameters dynamicParams;
  dynamicParams.steppingFrequency = 1.0 / 0.7;
  dynamicParams.swingRatio = 3.0 / 7.0;
  
  dynamicParams.phaseOffsets = {-currentPhase , -currentPhase , 0};

  const scalar_t slope = 0.0;
  const vector3_t terrainEulerZyx{0.0, slope, 0.0};
  const matrix3_t terrainRotation = getRotationMatrixFromZyxEulerAngles(terrainEulerZyx); 

  TerrainPlane slopyTerrain(vector3_t::Zero(), terrainRotation.transpose());

  PlanarTerrainModel terrainModel(slopyTerrain);

  BaseTrajectoryPlanner::StaticSettings staticSettings;
  staticSettings.deltaTime = deltaTime;
  scalar_t initialBaseHeight = 0.25 * std::sqrt(2.0);
  staticSettings.minimumBaseHeight = 0.1;
  staticSettings.maximumBaseHeight = 5.0;
  staticSettings.nominalBaseWidthHeading = 0.2;
  staticSettings.nominalBaseWidthLateral = 0.2;
  staticSettings.maximumBaseHeadingVelocity = 1.0;
  staticSettings.maximumBaseLateralVelocity = 1.0;
  staticSettings.maximumBaseVerticalVelocity = 1.0;
  staticSettings.maximumYawRate = 1.0;

  std::string urdfPathName = meldogWithBaseLinkUrdfFile;

  std::string solverName = "NewtonRaphson";

  KinematicsModelSettings modelSettings;
  modelSettings.baseLinkName = baseLink;
  modelSettings.threeDofEndEffectorNames = meldog3DofContactNames;

  InverseSolverSettings solverSettings;
  // solverSettings.stepCoefficient = 0.1;
  solverSettings.tolerance = 1e-3;
  MultiEndEffectorKinematics kinematicsSolver(urdfPathName, modelSettings, 
    solverSettings, solverName);

  ocs2::PinocchioInterface interface = createPinocchioInterfaceFromUrdfFile(urdfPathName, baseLink);
  const FloatingBaseModelInfo modelInfo = createFloatingBaseModelInfo(interface, meldog3DofContactNames, meldog6DofContactNames);
  
  BaseTrajectoryPlanner::BaseReferenceCommand command;
  command.baseHeadingVelocity = std::rand() / scalar_t(RAND_MAX);
  command.baseLateralVelocity = std::rand() / scalar_t(RAND_MAX);
  command.baseVerticalVelocity = 0.0;
  command.yawRate = std::rand() / scalar_t(RAND_MAX);
  
  std::string modelName = "meldogForwardKinematics";
  PinocchioForwardEndEffectorKinematicsCppAd forwardKinematics(interface, 
    modelInfo, modelName);

  SwingTrajectoryPlanner::StaticSettings swingStaticSettings;
  SwingTrajectoryPlanner::DynamicSettings swingDynamicSettings;
  swingDynamicSettings.phases.resize(4);
  swingDynamicSettings.swingHeights.resize(4);
  swingDynamicSettings.tangentialProgresses.resize(4);
  swingDynamicSettings.tangentialVelocityFactors.resize(4);
  for(size_t i = 0; i < 4; ++i)
  {
    swingDynamicSettings.phases[i] = 0.5;
    swingDynamicSettings.swingHeights[i] = 0.1;
    swingDynamicSettings.tangentialProgresses[i] = 0.5;
    swingDynamicSettings.tangentialVelocityFactors[i] = 2.0;
  }

  GaitPlanner gaitPlanner(staticParams, dynamicParams, defTime);

  GaitPlanner trueGaitPlanner = gaitPlanner;

  BaseTrajectoryPlanner basePlanner(modelInfo, staticSettings);

  ModelSettings leggedModelSettings;
  leggedModelSettings.baseLinkName =  baseLink;
  leggedModelSettings.endEffectorThreeDofNames = meldog3DofContactNames;
  leggedModelSettings.endEffectorSixDofNames = meldog6DofContactNames;
  leggedModelSettings.hipFrameNames = meldogHipNames;

  OverExtensionPenalty::Settings penaltySettings;
  penaltySettings.nominalLegExtension = 0.5;
  penaltySettings.legOverExtensionWeight = 1.0;

  const std::string penaltyName = "over_extension_penalty";

  OverExtensionPenalty penalty(interface, leggedModelSettings, penaltySettings, modelInfo, 
    penaltyName);

  SwingTrajectoryPlanner swingPlanner(modelInfo, swingStaticSettings, 
    swingDynamicSettings, forwardKinematics, penalty);

  JointTrajectoryPlanner jointPlanner(modelInfo, std::move(kinematicsSolver));

  PinocchioWeightCompensator compensator(interface, modelInfo);
  ContactForceWrenchTrajectoryPlanner forcePlanner(compensator);
  
  LeggedReferenceManager::Settings managerSettings;

  const auto modeSchedule = gaitPlanner.getModeSchedule(initTime, finalTime);
  contact_flags_t contactFlag = modeSchedule.modeAtTime(initTime);

  LeggedReferenceManager manager(modelInfo, 
    managerSettings, std::move(gaitPlanner), std::move(swingPlanner), std::move(basePlanner), 
    std::move(jointPlanner), std::move(forcePlanner));

  const vector3_t initPosition = vector3_t{0.0, 0.0, 0.0};

  const vector3_t planeLocalEulerZyx = vector3_t{0.0, 0.0, 0.0};

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

  auto& initialState = initialObservation.state;

  access_helper_functions::getJointPositions(initialState, modelInfo) << 0, -0.785398163, 1.570796326, 0, -0.785398163, 1.570796326, 0, -0.785398163, 1.570796326, 0, -0.785398163, 1.570796326;

  std::unique_ptr<TerrainModel> terrainModelPtr = 
    std::make_unique<PlanarTerrainModel>(slopyTerrain);
  
  manager.initialize(initTime, finalTime, initialObservation, contactFlag, 
    dynamicParams, swingDynamicSettings, std::move(terrainModelPtr));
  
  const TerrainModel& terrain = manager.getTerrainModel();

  for(size_t i = 0; i < TEST_NUM; ++i)
  {
    const vector3_t positon = vector3_t::Random() * 100.0;
    const vector2_t position2D{positon.x(), positon.y()};
    EXPECT_TRUE((slopyTerrain.projectPositionInWorldOntoPlane(positon) - 
      terrain.getSmoothedPositon(position2D)).norm() < eps);
  }
}

TEST(LeggedReferenceManagerTest, getEndEffectorTrajectoryPoint) 
{
  const scalar_t defTime = 0.0;
  const scalar_t initTime = 0.0;
  const scalar_t finalTime = 2.1;
  const scalar_t deltaTime = 0.01;

  /* STANDING TROT */
  scalar_t currentPhase = 3.5 / 7.0;

  GaitStaticParameters staticParams;
  staticParams.endEffectorNumber = 4;
  staticParams.initPhase = currentPhase;

  GaitDynamicParameters dynamicParams;
  dynamicParams.steppingFrequency = 1.0 / 0.7;
  dynamicParams.swingRatio = 3.0 / 7.0;
  
  dynamicParams.phaseOffsets = {-currentPhase , -currentPhase , 0};

  const scalar_t slope = 0.0;
  const vector3_t terrainEulerZyx{0.0, slope, 0.0};
  const matrix3_t terrainRotation = getRotationMatrixFromZyxEulerAngles(terrainEulerZyx); 

  TerrainPlane slopyTerrain(vector3_t::Zero(), terrainRotation.transpose());

  PlanarTerrainModel terrainModel(slopyTerrain);

  BaseTrajectoryPlanner::StaticSettings staticSettings;
  staticSettings.deltaTime = deltaTime;
  scalar_t initialBaseHeight = 0.25 * std::sqrt(2.0);
  staticSettings.minimumBaseHeight = 0.1;
  staticSettings.maximumBaseHeight = 5.0;
  staticSettings.nominalBaseWidthHeading = 0.2;
  staticSettings.nominalBaseWidthLateral = 0.2;
  staticSettings.maximumBaseHeadingVelocity = 1.0;
  staticSettings.maximumBaseLateralVelocity = 1.0;
  staticSettings.maximumBaseVerticalVelocity = 1.0;
  staticSettings.maximumYawRate = 1.0;

  std::string urdfPathName = meldogWithBaseLinkUrdfFile;

  std::string solverName = "NewtonRaphson";

  KinematicsModelSettings modelSettings;
  modelSettings.baseLinkName = baseLink;
  modelSettings.threeDofEndEffectorNames = meldog3DofContactNames;

  InverseSolverSettings solverSettings;
  // solverSettings.stepCoefficient = 0.1;
  solverSettings.tolerance = 1e-3;
  MultiEndEffectorKinematics kinematicsSolver(urdfPathName, modelSettings, 
    solverSettings, solverName);

  ocs2::PinocchioInterface interface = createPinocchioInterfaceFromUrdfFile(urdfPathName, baseLink);
  const FloatingBaseModelInfo modelInfo = createFloatingBaseModelInfo(interface, meldog3DofContactNames, meldog6DofContactNames);
  
  BaseTrajectoryPlanner::BaseReferenceCommand command;
  command.baseHeadingVelocity = std::rand() / scalar_t(RAND_MAX);
  command.baseLateralVelocity = std::rand() / scalar_t(RAND_MAX);
  command.baseVerticalVelocity = 0.0;
  command.yawRate = std::rand() / scalar_t(RAND_MAX);
  
  std::string modelName = "meldogForwardKinematics";
  PinocchioForwardEndEffectorKinematicsCppAd forwardKinematics(interface, 
    modelInfo, modelName);

  SwingTrajectoryPlanner::StaticSettings swingStaticSettings;
  SwingTrajectoryPlanner::DynamicSettings swingDynamicSettings;
  swingDynamicSettings.phases.resize(4);
  swingDynamicSettings.swingHeights.resize(4);
  swingDynamicSettings.tangentialProgresses.resize(4);
  swingDynamicSettings.tangentialVelocityFactors.resize(4);
  for(size_t i = 0; i < 4; ++i)
  {
    swingDynamicSettings.phases[i] = 0.5;
    swingDynamicSettings.swingHeights[i] = 0.1;
    swingDynamicSettings.tangentialProgresses[i] = 0.5;
    swingDynamicSettings.tangentialVelocityFactors[i] = 2.0;
  }

  GaitPlanner gaitPlanner(staticParams, dynamicParams, defTime);

  GaitPlanner trueGaitPlanner = gaitPlanner;

  BaseTrajectoryPlanner basePlanner(modelInfo, staticSettings);

  ModelSettings leggedModelSettings;
  leggedModelSettings.baseLinkName =  baseLink;
  leggedModelSettings.endEffectorThreeDofNames = meldog3DofContactNames;
  leggedModelSettings.endEffectorSixDofNames = meldog6DofContactNames;
  leggedModelSettings.hipFrameNames = meldogHipNames;

  OverExtensionPenalty::Settings penaltySettings;
  penaltySettings.nominalLegExtension = 0.5;
  penaltySettings.legOverExtensionWeight = 1.0;

  const std::string penaltyName = "over_extension_penalty";

  OverExtensionPenalty penalty(interface, leggedModelSettings, penaltySettings, modelInfo, 
    penaltyName);

  SwingTrajectoryPlanner swingPlanner(modelInfo, swingStaticSettings, 
    swingDynamicSettings, forwardKinematics, penalty);

  JointTrajectoryPlanner jointPlanner(modelInfo, std::move(kinematicsSolver));

  PinocchioWeightCompensator compensator(interface, modelInfo);
  ContactForceWrenchTrajectoryPlanner forcePlanner(compensator);
  
  LeggedReferenceManager::Settings managerSettings;

  const auto modeSchedule = gaitPlanner.getModeSchedule(initTime, finalTime);
  contact_flags_t contactFlag = modeSchedule.modeAtTime(initTime);

  LeggedReferenceManager manager(modelInfo, 
    managerSettings, std::move(gaitPlanner), std::move(swingPlanner), std::move(basePlanner), 
    std::move(jointPlanner), std::move(forcePlanner));

  const vector3_t initPosition = vector3_t{0.0, 0.0, 0.0};

  const vector3_t planeLocalEulerZyx = vector3_t{0.0, 0.0, 0.0};

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

  auto& initialState = initialObservation.state;

  access_helper_functions::getJointPositions(initialState, modelInfo) << 0, -0.785398163, 1.570796326, 0, -0.785398163, 1.570796326, 0, -0.785398163, 1.570796326, 0, -0.785398163, 1.570796326;

  std::unique_ptr<TerrainModel> terrainModelPtr = 
    std::make_unique<PlanarTerrainModel>(slopyTerrain);
  
  manager.initialize(initTime, finalTime, initialObservation, contactFlag, 
    dynamicParams, swingDynamicSettings, std::move(terrainModelPtr));
  
  std::vector<scalar_t> alphaValues{0.0, 0.1, 0.2, 0.5, 0.7, 1.0};

  for(const auto alpha: alphaValues)
  {
    scalar_t testTime = 0.0;
    scalar_t testDeltaTime = deltaTime * alpha;
    
    while((testTime + testDeltaTime) < finalTime)
    {
      const auto point1 = manager.getEndEffectorTrajectoryPoint(testTime);
      const auto point2 = manager.getEndEffectorTrajectoryPoint(testTime + deltaTime);
      const auto pointMiddle = manager.getEndEffectorTrajectoryPoint(testTime + testDeltaTime);
      EXPECT_TRUE(point1.positions.size() == 4);
      EXPECT_TRUE(point1.velocities.size() == 4);
      EXPECT_TRUE(point1.clearances.size() == 4);
      for(size_t i = 0; i < point1.positions.size(); ++i)
      {
        const vector3_t truePosition = (1 - alpha) * point1.positions[i] + alpha * point2.positions[i];
        const vector3_t trueVelocity = (1 - alpha) * point1.velocities[i] + alpha * point2.velocities[i];
        const scalar_t trueClearance = (1 - alpha) * point1.clearances[i] + alpha * point2.clearances[i];
        const vector3_t trueNormal = (1 - alpha) * point1.surfaceNormals[i] + alpha * point2.surfaceNormals[i];
        EXPECT_TRUE((pointMiddle.positions[i] - truePosition).norm() < eps);
        EXPECT_TRUE((pointMiddle.velocities[i] - trueVelocity).norm() < eps);
        EXPECT_TRUE(std::abs(pointMiddle.clearances[i] - trueClearance) < eps);
        EXPECT_TRUE((pointMiddle.surfaceNormals[i] - trueNormal).norm() < eps);
      }
      testTime += managerSettings.maximumReferenceSampleInterval;
    }
  } 
}

TEST(LeggedReferenceManagerTest, getEndEffectorConstraintMatrixes) 
{
  const scalar_t defTime = 0.0;
  const scalar_t initTime = 0.0;
  const scalar_t finalTime = 2.1;
  const scalar_t deltaTime = 0.01;

  /* STANDING TROT */
  scalar_t currentPhase = 3.5 / 7.0;

  GaitStaticParameters staticParams;
  staticParams.endEffectorNumber = 4;
  staticParams.initPhase = currentPhase;
  
  GaitDynamicParameters dynamicParams;
  dynamicParams.steppingFrequency = 1.0 / 0.7;
  dynamicParams.swingRatio = 3.0 / 7.0;
  
  dynamicParams.phaseOffsets = {-currentPhase , -currentPhase , 0};

  const scalar_t slope = 0.0;
  const vector3_t terrainEulerZyx{0.0, slope, 0.0};
  const matrix3_t terrainRotation = getRotationMatrixFromZyxEulerAngles(terrainEulerZyx); 

  TerrainPlane slopyTerrain(vector3_t::Zero(), terrainRotation.transpose());

  PlanarTerrainModel terrainModel(slopyTerrain);

  BaseTrajectoryPlanner::StaticSettings staticSettings;
  staticSettings.deltaTime = deltaTime;
  scalar_t initialBaseHeight = 0.25 * std::sqrt(2.0);
  staticSettings.minimumBaseHeight = 0.1;
  staticSettings.maximumBaseHeight = 5.0;
  staticSettings.nominalBaseWidthHeading = 0.2;
  staticSettings.nominalBaseWidthLateral = 0.2;
  staticSettings.maximumBaseHeadingVelocity = 1.0;
  staticSettings.maximumBaseLateralVelocity = 1.0;
  staticSettings.maximumBaseVerticalVelocity = 1.0;
  staticSettings.maximumYawRate = 1.0;

  std::string urdfPathName = meldogWithBaseLinkUrdfFile;

  std::string solverName = "NewtonRaphson";

  KinematicsModelSettings modelSettings;
  modelSettings.baseLinkName = baseLink;
  modelSettings.threeDofEndEffectorNames = meldog3DofContactNames;

  InverseSolverSettings solverSettings;
  // solverSettings.stepCoefficient = 0.1;
  solverSettings.tolerance = 1e-3;
  MultiEndEffectorKinematics kinematicsSolver(urdfPathName, modelSettings, 
    solverSettings, solverName);

  ocs2::PinocchioInterface interface = createPinocchioInterfaceFromUrdfFile(urdfPathName, baseLink);
  const FloatingBaseModelInfo modelInfo = createFloatingBaseModelInfo(interface, meldog3DofContactNames, meldog6DofContactNames);
  
  BaseTrajectoryPlanner::BaseReferenceCommand command;
  command.baseHeadingVelocity = std::rand() / scalar_t(RAND_MAX);
  command.baseLateralVelocity = std::rand() / scalar_t(RAND_MAX);
  command.baseVerticalVelocity = 0.0;
  command.yawRate = std::rand() / scalar_t(RAND_MAX);
  
  std::string modelName = "meldogForwardKinematics";
  PinocchioForwardEndEffectorKinematicsCppAd forwardKinematics(interface, 
    modelInfo, modelName);

  SwingTrajectoryPlanner::StaticSettings swingStaticSettings;
  SwingTrajectoryPlanner::DynamicSettings swingDynamicSettings;
  swingDynamicSettings.phases.resize(4);
  swingDynamicSettings.swingHeights.resize(4);
  swingDynamicSettings.tangentialProgresses.resize(4);
  swingDynamicSettings.tangentialVelocityFactors.resize(4);
  for(size_t i = 0; i < 4; ++i)
  {
    swingDynamicSettings.phases[i] = 0.5;
    swingDynamicSettings.swingHeights[i] = 0.1;
    swingDynamicSettings.tangentialProgresses[i] = 0.5;
    swingDynamicSettings.tangentialVelocityFactors[i] = 2.0;
  }

  GaitPlanner gaitPlanner(staticParams, dynamicParams, defTime);

  GaitPlanner trueGaitPlanner = gaitPlanner;

  BaseTrajectoryPlanner basePlanner(modelInfo, staticSettings);

  ModelSettings leggedModelSettings;
  leggedModelSettings.baseLinkName =  baseLink;
  leggedModelSettings.endEffectorThreeDofNames = meldog3DofContactNames;
  leggedModelSettings.endEffectorSixDofNames = meldog6DofContactNames;
  leggedModelSettings.hipFrameNames = meldogHipNames;

  OverExtensionPenalty::Settings penaltySettings;
  penaltySettings.nominalLegExtension = 0.5;
  penaltySettings.legOverExtensionWeight = 1.0;

  const std::string penaltyName = "over_extension_penalty";

  OverExtensionPenalty penalty(interface, leggedModelSettings, penaltySettings, modelInfo, 
    penaltyName);

  SwingTrajectoryPlanner swingPlanner(modelInfo, swingStaticSettings, 
    swingDynamicSettings, forwardKinematics, penalty);

  JointTrajectoryPlanner jointPlanner(modelInfo, std::move(kinematicsSolver));

  PinocchioWeightCompensator compensator(interface, modelInfo);
  ContactForceWrenchTrajectoryPlanner forcePlanner(compensator);
  
  LeggedReferenceManager::Settings managerSettings;

  const auto modeSchedule = gaitPlanner.getModeSchedule(initTime, finalTime);
  contact_flags_t contactFlag = modeSchedule.modeAtTime(initTime);

  LeggedReferenceManager manager(modelInfo, 
    managerSettings, std::move(gaitPlanner), std::move(swingPlanner), std::move(basePlanner), 
    std::move(jointPlanner), std::move(forcePlanner));

  const vector3_t initPosition = vector3_t{0.0, 0.0, 0.0};

  const vector3_t planeLocalEulerZyx = vector3_t{0.0, 0.0, 0.0};

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

  auto& initialState = initialObservation.state;

  access_helper_functions::getJointPositions(initialState, modelInfo) << 0, -0.785398163, 1.570796326, 0, -0.785398163, 1.570796326, 0, -0.785398163, 1.570796326, 0, -0.785398163, 1.570796326;

  std::unique_ptr<TerrainModel> terrainModelPtr = 
    std::make_unique<PlanarTerrainModel>(slopyTerrain);
  
  manager.initialize(initTime, finalTime, initialObservation, contactFlag, 
    dynamicParams, swingDynamicSettings, std::move(terrainModelPtr));

  std::vector<scalar_t> goodTimings = {0.0, 0.3, 0.35, 0.65, 0.7, 1, 1.05, 1.35, 1.4, 1.7, 1.75, 2.05, 2.1, 2.4};
  std::vector<size_t> goodSequence = {15, 9, 15, 6, 15, 9, 15, 6, 15, 9, 15, 6, 15, 9, 15};
  const auto mySequence = manager.getModeSchedule().modeSequence;

  EXPECT_TRUE(mySequence == goodSequence);

  scalar_t testTime = initTime;
  scalar_t testDeltaTime = managerSettings.maximumReferenceSampleInterval;

  std::vector<bool> isInFirstContact(staticParams.endEffectorNumber);
  for(size_t i = 0; i < staticParams.endEffectorNumber; ++i)
  {
    isInFirstContact[i] = contactFlag[i];
  }
    
  while((testTime + testDeltaTime) < finalTime)
  {
    const auto contactFlags = manager.getContactFlags(testTime);
    const auto& constraints  = manager.getEndEffectorConstraintMatrixes(testTime);
    for(size_t i = 0; i < staticParams.endEffectorNumber; ++i)
    {
      // When robot leg is already in contact at the start, constraint does not work
      if(contactFlags[i] && isInFirstContact[i])  EXPECT_TRUE(!constraints[i].isActive() && isInFirstContact[i]);
      
      // When robot legs in in motion after first contact, constraint does not work
      if(!contactFlags[i] && isInFirstContact[i])  
      {
        isInFirstContact[i] = !isInFirstContact[i];
        EXPECT_TRUE(!constraints[i].isActive());
      }

      // Later contacts (after first one, if leg started from contact)
      if(contactFlags[i] && !isInFirstContact[i])
      {
        EXPECT_TRUE(constraints[i].isActive() && constraints[i].A.rows() == 3);
      }

      // Later motions (after first one, if leg started from contact)
      if(!contactFlags[i] && !isInFirstContact[i])
      {
        EXPECT_TRUE(!constraints[i].isActive() && !isInFirstContact[i]);
      }
    }
    testTime += testDeltaTime;
  }
}

TEST(LeggedReferenceManagerTest, loader)
{
  const std::string filePath = meldogConfigFolder + "reference_manager_settings.info";

  const auto referenceSettings = loadLeggedReferenceManagerSettings(filePath);

  EXPECT_TRUE(referenceSettings.maximumReferenceSampleInterval == 0.5);
  EXPECT_TRUE(referenceSettings.gaitUpdateOffset == 0.25);
  EXPECT_TRUE(referenceSettings.threaded == false);
}