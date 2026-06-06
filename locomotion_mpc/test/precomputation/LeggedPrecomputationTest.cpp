#include <gtest/gtest.h>

#include <legged_locomotion_mpc/precomputation/LeggedPrecomputation.hpp>
#include <legged_locomotion_mpc/reference_manager/LeggedReferenceManager.hpp>
#include <legged_locomotion_mpc/collision/PinocchioForwardCollisionKinematicsCppAd.hpp>
#include <legged_locomotion_mpc/torque_approx/PinocchioTorqueApproximationCppAd.hpp>


#include <floating_base_model/FactoryFunctions.hpp>

#include <terrain_model/core/TerrainPlane.hpp>
#include <terrain_model/planar_model/PlanarTerrainModel.hpp>

#include "../test/include/definitions.hpp"

using namespace legged_locomotion_mpc;
using namespace legged_locomotion_mpc::locomotion;
using namespace legged_locomotion_mpc::planners;
using namespace legged_locomotion_mpc::collision;
using namespace floating_base_model;
using namespace ocs2;
using namespace terrain_model;
using namespace multi_end_effector_kinematics;

const double eps = 1e-9;
const size_t TEST_NUM = 100;

// Not round because std::lower_bound is sensitive for time points of mode change
const size_t ITERATIONS = 50;

TEST(LeggedPrecomputationTest, getEndEffector) 
{
  const scalar_t defTime = 0.0;
  const scalar_t initTime = 0.0;
  const scalar_t finalTime = 2.1;
  const scalar_t deltaTime = 0.1;

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

  const std::vector<std::string> meldogFake3DofContactNames = {"RFF_link", "RRF_link", "LFF_link", "LRF_link"};
  const std::vector<std::string> meldogFake6DofContactNames = {};

  KinematicsModelSettings modelSettings;
  modelSettings.baseLinkName = baseLink;
  modelSettings.threeDofEndEffectorNames = meldog3DofContactNames;

  InverseSolverSettings solverSettings;
  // solverSettings.stepCoefficient = 0.1;
  solverSettings.tolerance = 1e-3;
  MultiEndEffectorKinematics kinematicsSolver(urdfPathName, modelSettings, 
    solverSettings, solverName);

  ocs2::PinocchioInterface interface = createPinocchioInterfaceFromUrdfFile(urdfPathName, baseLink);
  const FloatingBaseModelInfo modelInfo = createFloatingBaseModelInfo(interface, meldogFake3DofContactNames, meldogFake6DofContactNames);
  
  BaseTrajectoryPlanner::BaseReferenceCommand command;
  command.baseHeadingVelocity = std::rand() / scalar_t(RAND_MAX);
  command.baseLateralVelocity = std::rand() / scalar_t(RAND_MAX);
  command.baseVerticalVelocity = 0.0;
  command.yawRate = std::rand() / scalar_t(RAND_MAX);
  
  std::string kinematicsModelName = "meldogForwardKinematics";
  PinocchioForwardEndEffectorKinematicsCppAd forwardKinematics(interface, 
    modelInfo, kinematicsModelName);

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

  CollisionSettings collisionSettings;
  collisionSettings.collisionLinkNames = meldogCollisions;
  collisionSettings.terrainCollisionLinkNames = {"LFLL_link", "RFLL_link"};
  collisionSettings.selfCollisionPairNames = {{"LFLL_link", "RFLL_link"}, {"LFLL_link", "LRLL_link"}, {"RRLL_link", "RRF_link"}};
  collisionSettings.maxExcesses = std::vector<scalar_t>(
    meldog3DofContactNames.size() + collisionSettings.collisionLinkNames.size(), 0.2);
  collisionSettings.terrainRelaxations = std::vector<scalar_t>(
    meldog3DofContactNames.size() + collisionSettings.terrainCollisionLinkNames.size(), 0.5);
  collisionSettings.selfCollisionRelaxations = {0.5, 0.5};
  collisionSettings.shrinkRatio = 0.5;

  const std::string collisionModelName = "collision_kinematics";

  PinocchioForwardCollisionKinematicsCppAd collisionKinematics(interface, modelInfo, 
    collisionSettings, collisionModelName);

  const std::string torqueModelName = "torque_approx";
  PinocchioTorqueApproximationCppAd torqueApproximator(interface, modelInfo,
    vector_t::Zero(modelInfo.actuatedDofNum), torqueModelName);

  LeggedPrecomputation precomputation(modelInfo, manager, forwardKinematics, 
    collisionKinematics, torqueApproximator, compensator);

  scalar_t testTime = 0.0;
  scalar_t testDeltaTime = 0.5 * deltaTime;
    
  for(size_t iter = 0; iter < ITERATIONS; ++iter)
  {
    const vector_t state = vector_t::Random(modelInfo.stateDim);
    const vector_t input = vector_t::Random(modelInfo.inputDim);
    RequestSet set{Request::Constraint + Request::Approximation};
    precomputation.request(set, testTime, state, input);

    const auto truePositions = forwardKinematics.getPosition(state);
    const auto truePositionDerivatives = forwardKinematics.getPositionLinearApproximation(state);
    const auto trueLinearVelocities = forwardKinematics.getLinearVelocity(state, input);
    const auto trueLinearVelocityDerivatives = forwardKinematics.getLinearVelocityLinearApproximation(state, input);

    for(size_t i = 0; i < 4; ++i)
    {
      const auto& position = precomputation.getEndEffectorPosition(i);
      const auto& positionDerivative = precomputation.getEndEffectorPositionDerivatives(i);
      const auto& linearVelocity = precomputation.getEndEffectorLinearVelocity(i);
      const auto& linearVelocityDerivative = precomputation.getEndEffectorLinearVelocityDerivatives(i);
      
      EXPECT_TRUE((truePositions[i] - position).norm() < eps);
      EXPECT_TRUE((trueLinearVelocities[i] - linearVelocity).norm() < eps);
      EXPECT_TRUE((truePositionDerivatives[i].dfdx - positionDerivative.dfdx).norm() < eps);
      EXPECT_TRUE((truePositionDerivatives[i].dfdu - positionDerivative.dfdu).norm() < eps);
      EXPECT_TRUE((trueLinearVelocityDerivatives[i].dfdx - linearVelocityDerivative.dfdx).norm() < eps);
      EXPECT_TRUE((trueLinearVelocityDerivatives[i].dfdu - linearVelocityDerivative.dfdu).norm() < eps);
    }
    testTime += testDeltaTime;
  }
}

TEST(LeggedPrecomputationTest, getCollisionLinks) 
{
  const scalar_t defTime = 0.0;
  const scalar_t initTime = 0.0;
  const scalar_t finalTime = 2.1;
  const scalar_t deltaTime = 0.1;

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

  const std::vector<std::string> meldogFake3DofContactNames = {"RFF_link", "RRF_link", "LFF_link", "LRF_link"};
  const std::vector<std::string> meldogFake6DofContactNames = {};

  KinematicsModelSettings modelSettings;
  modelSettings.baseLinkName = baseLink;
  modelSettings.threeDofEndEffectorNames = meldog3DofContactNames;

  InverseSolverSettings solverSettings;
  // solverSettings.stepCoefficient = 0.1;
  solverSettings.tolerance = 1e-3;
  MultiEndEffectorKinematics kinematicsSolver(urdfPathName, modelSettings, 
    solverSettings, solverName);

  ocs2::PinocchioInterface interface = createPinocchioInterfaceFromUrdfFile(urdfPathName, baseLink);
  const FloatingBaseModelInfo modelInfo = createFloatingBaseModelInfo(interface, meldogFake3DofContactNames, meldogFake6DofContactNames);
  
  BaseTrajectoryPlanner::BaseReferenceCommand command;
  command.baseHeadingVelocity = std::rand() / scalar_t(RAND_MAX);
  command.baseLateralVelocity = std::rand() / scalar_t(RAND_MAX);
  command.baseVerticalVelocity = 0.0;
  command.yawRate = std::rand() / scalar_t(RAND_MAX);
  
  std::string kinematicsModelName = "meldogForwardKinematics";
  PinocchioForwardEndEffectorKinematicsCppAd forwardKinematics(interface, 
    modelInfo, kinematicsModelName);

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
  
  initialBasePosition.z() += initialBaseHeight/ slopyTerrain.getSurfaceNormalInWorld().z();
  
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

  CollisionSettings collisionSettings;
  collisionSettings.collisionLinkNames = meldogCollisions;
  collisionSettings.terrainCollisionLinkNames = {"LFLL_link", "RFLL_link"};
  collisionSettings.selfCollisionPairNames = {{"LFLL_link", "RFLL_link"}, {"LFLL_link", "LRLL_link"}, {"RRLL_link", "RRF_link"}};
  collisionSettings.maxExcesses = std::vector<scalar_t>(
    meldog3DofContactNames.size() + collisionSettings.collisionLinkNames.size(), 0.2);
  collisionSettings.terrainRelaxations = std::vector<scalar_t>(
    meldog3DofContactNames.size() + collisionSettings.terrainCollisionLinkNames.size(), 0.5);
  collisionSettings.selfCollisionRelaxations = {0.5, 0.5};
  collisionSettings.shrinkRatio = 0.5;

  const std::string collisionModelName = "collision_kinematics";

  PinocchioForwardCollisionKinematicsCppAd collisionKinematics(interface, modelInfo, 
    collisionSettings, collisionModelName);

  const std::string torqueModelName = "torque_approx";
  PinocchioTorqueApproximationCppAd torqueApproximator(interface, modelInfo,
    vector_t::Zero(modelInfo.actuatedDofNum), torqueModelName);

  LeggedPrecomputation precomputation(modelInfo, manager, forwardKinematics, 
    collisionKinematics, torqueApproximator, compensator);

  scalar_t testTime = 0.0;
  scalar_t testDeltaTime = 0.5 * deltaTime;
    
  for(size_t iter = 0; iter < ITERATIONS; ++iter)
  {
    const vector_t state = vector_t::Random(modelInfo.stateDim);
    const vector_t input = vector_t::Random(modelInfo.inputDim);
    RequestSet set{Request::SoftConstraint + Request::Approximation};
    precomputation.request(set, testTime, state, input);

    const auto truePositions = collisionKinematics.getPosition(state);
    const auto truePositionDerivatives = collisionKinematics.getPositionLinearApproximation(state);
    const auto trueEulerAngles = collisionKinematics.getOrientation(state);
    const auto trueEulerAngleDerivatives = collisionKinematics.getOrientationLinearApproximation(state);

    for(size_t i = 0; i < collisionSettings.collisionLinkNames.size(); ++i)
    {
      const size_t collisionIndex = i + 4;
      const auto& position = precomputation.getCollisionLinkPosition(collisionIndex);
      const auto& positionDerivative = precomputation.getCollisionLinkPositionDerivatives(collisionIndex);
      const auto& eulerAngles = precomputation.getCollisionLinkOrientation(collisionIndex);
      const auto& eulerAngleDerivative = precomputation.getCollisionLinkOrientationDerivatives(collisionIndex);

      EXPECT_TRUE((truePositions[i] - position).norm() < eps);
      EXPECT_TRUE((trueEulerAngles[i] - eulerAngles).norm() < eps);
      EXPECT_TRUE((truePositionDerivatives[i].dfdx - positionDerivative.dfdx).norm() < eps);
      EXPECT_TRUE((truePositionDerivatives[i].dfdu - positionDerivative.dfdu).norm() < eps);
      EXPECT_TRUE((trueEulerAngleDerivatives[i].dfdx - eulerAngleDerivative.dfdx).norm() < eps);
      EXPECT_TRUE((trueEulerAngleDerivatives[i].dfdu - eulerAngleDerivative.dfdu).norm() < eps);
    }
    testTime += testDeltaTime;
  }
}

TEST(LeggedPrecomputationTest, getTorque) 
{
  const scalar_t defTime = 0.0;
  const scalar_t initTime = 0.0;
  const scalar_t finalTime = 2.1;
  const scalar_t deltaTime = 0.1;

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

  const std::vector<std::string> meldogFake3DofContactNames = {"RFF_link", "RRF_link", "LFF_link", "LRF_link"};
  const std::vector<std::string> meldogFake6DofContactNames = {};

  KinematicsModelSettings modelSettings;
  modelSettings.baseLinkName = baseLink;
  modelSettings.threeDofEndEffectorNames = meldog3DofContactNames;

  InverseSolverSettings solverSettings;
  // solverSettings.stepCoefficient = 0.1;
  solverSettings.tolerance = 1e-3;
  MultiEndEffectorKinematics kinematicsSolver(urdfPathName, modelSettings, 
    solverSettings, solverName);

  ocs2::PinocchioInterface interface = createPinocchioInterfaceFromUrdfFile(urdfPathName, baseLink);
  const FloatingBaseModelInfo modelInfo = createFloatingBaseModelInfo(interface, meldogFake3DofContactNames, meldogFake6DofContactNames);
  
  BaseTrajectoryPlanner::BaseReferenceCommand command;
  command.baseHeadingVelocity = std::rand() / scalar_t(RAND_MAX);
  command.baseLateralVelocity = std::rand() / scalar_t(RAND_MAX);
  command.baseVerticalVelocity = 0.0;
  command.yawRate = std::rand() / scalar_t(RAND_MAX);
  
  std::string kinematicsModelName = "meldogForwardKinematics";
  PinocchioForwardEndEffectorKinematicsCppAd forwardKinematics(interface, 
    modelInfo, kinematicsModelName);

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
  
  initialBasePosition.z() += initialBaseHeight/ slopyTerrain.getSurfaceNormalInWorld().z();
  
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

  CollisionSettings collisionSettings;
  collisionSettings.collisionLinkNames = meldogCollisions;
  collisionSettings.terrainCollisionLinkNames = {"LFLL_link", "RFLL_link"};
  collisionSettings.selfCollisionPairNames = {{"LFLL_link", "RFLL_link"}, {"LFLL_link", "LRLL_link"}, {"RRLL_link", "RRF_link"}};
  collisionSettings.maxExcesses = std::vector<scalar_t>(
    meldog3DofContactNames.size() + collisionSettings.collisionLinkNames.size(), 0.2);
  collisionSettings.terrainRelaxations = std::vector<scalar_t>(
    meldog3DofContactNames.size() + collisionSettings.terrainCollisionLinkNames.size(), 0.5);
  collisionSettings.selfCollisionRelaxations = {0.5, 0.5};
  collisionSettings.shrinkRatio = 0.5;

  const std::string collisionModelName = "collision_kinematics";

  PinocchioForwardCollisionKinematicsCppAd collisionKinematics(interface, modelInfo, 
    collisionSettings, collisionModelName);

  const std::string torqueModelName = "torque_approx";
  PinocchioTorqueApproximationCppAd torqueApproximator(interface, modelInfo,
    vector_t::Zero(modelInfo.actuatedDofNum), torqueModelName);

  LeggedPrecomputation precomputation(modelInfo, manager, forwardKinematics, 
    collisionKinematics, torqueApproximator, compensator);

  scalar_t testTime = 0.0;
  scalar_t testDeltaTime = 0.5 * deltaTime;
    
  for(size_t iter = 0; iter < ITERATIONS; ++iter)
  {
    const vector_t state = vector_t::Random(modelInfo.stateDim);
    const vector_t input = vector_t::Random(modelInfo.inputDim);
    RequestSet set{Request::SoftConstraint + Request::Approximation};
    precomputation.request(set, testTime, state, input);

    const auto trueTorque = torqueApproximator.getValue(state, input);
    const auto trueTorqueDerivative = torqueApproximator.getLinearApproximation(state, input);
    const auto& torque = precomputation.getApproximatedJointTorques();
    const auto& torqueDerivative = precomputation.getApproximatedJointTorquesDerivatives();

    EXPECT_TRUE((torque - trueTorque).norm() < eps);
    EXPECT_TRUE((trueTorqueDerivative.dfdx - torqueDerivative.dfdx).norm() < eps);
    EXPECT_TRUE((trueTorqueDerivative.dfdu - torqueDerivative.dfdu).norm() < eps);
    testTime += testDeltaTime;
  }
}

TEST(LeggedPrecomputationTest, getReference) 
{
  const scalar_t defTime = 0.0;
  const scalar_t initTime = 0.0;
  const scalar_t finalTime = 2.1;
  const scalar_t deltaTime = 0.1;

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

  const std::vector<std::string> meldogFake3DofContactNames = {"RFF_link", "RRF_link", "LFF_link", "LRF_link"};
  const std::vector<std::string> meldogFake6DofContactNames = {};

  KinematicsModelSettings modelSettings;
  modelSettings.baseLinkName = baseLink;
  modelSettings.threeDofEndEffectorNames = meldog3DofContactNames;

  InverseSolverSettings solverSettings;
  // solverSettings.stepCoefficient = 0.1;
  solverSettings.tolerance = 1e-3;
  MultiEndEffectorKinematics kinematicsSolver(urdfPathName, modelSettings, 
    solverSettings, solverName);

  ocs2::PinocchioInterface interface = createPinocchioInterfaceFromUrdfFile(urdfPathName, baseLink);
  const FloatingBaseModelInfo modelInfo = createFloatingBaseModelInfo(interface, meldogFake3DofContactNames, meldogFake6DofContactNames);
  
  BaseTrajectoryPlanner::BaseReferenceCommand command;
  command.baseHeadingVelocity = std::rand() / scalar_t(RAND_MAX);
  command.baseLateralVelocity = std::rand() / scalar_t(RAND_MAX);
  command.baseVerticalVelocity = 0.0;
  command.yawRate = std::rand() / scalar_t(RAND_MAX);
  
  std::string kinematicsModelName = "meldogForwardKinematics";
  PinocchioForwardEndEffectorKinematicsCppAd forwardKinematics(interface, 
    modelInfo, kinematicsModelName);

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
  
  initialBasePosition.z() += initialBaseHeight/ slopyTerrain.getSurfaceNormalInWorld().z();
  
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

  CollisionSettings collisionSettings;
  collisionSettings.collisionLinkNames = meldogCollisions;
  collisionSettings.terrainCollisionLinkNames = {"LFLL_link", "RFLL_link"};
  collisionSettings.selfCollisionPairNames = {{"LFLL_link", "RFLL_link"}, {"LFLL_link", "LRLL_link"}, {"RRLL_link", "RRF_link"}};
  collisionSettings.maxExcesses = std::vector<scalar_t>(
    meldog3DofContactNames.size() + collisionSettings.collisionLinkNames.size(), 0.2);
  collisionSettings.terrainRelaxations = std::vector<scalar_t>(
    meldog3DofContactNames.size() + collisionSettings.terrainCollisionLinkNames.size(), 0.5);
  collisionSettings.selfCollisionRelaxations = {0.5, 0.5};
  collisionSettings.shrinkRatio = 0.5;

  const std::string collisionModelName = "collision_kinematics";

  PinocchioForwardCollisionKinematicsCppAd collisionKinematics(interface, modelInfo, 
    collisionSettings, collisionModelName);

  const std::string torqueModelName = "torque_approx";
  PinocchioTorqueApproximationCppAd torqueApproximator(interface, modelInfo,
    vector_t::Zero(modelInfo.actuatedDofNum), torqueModelName);

  LeggedPrecomputation precomputation(modelInfo, manager, forwardKinematics, 
    collisionKinematics, torqueApproximator, compensator);

  scalar_t testTime = 0.0;
  scalar_t testDeltaTime = 0.5 * deltaTime;
    
  while(testTime < finalTime)
  {
    const vector_t state = vector_t::Random(modelInfo.stateDim);
    const vector_t input = vector_t::Random(modelInfo.inputDim);
    RequestSet set{Request::SoftConstraint + Request::Approximation};
    precomputation.request(set, testTime, state, input);

    const auto trueReference = manager.getEndEffectorTrajectoryPoint(testTime);
    const auto& truePositons = trueReference.positions;
    const auto& trueVelocities = trueReference.velocities;
    const auto& trueClearances = trueReference.clearances;
    const auto& trueNormals = trueReference.surfaceNormals;

    for(size_t i = 0; i < 4; ++i)
    {
      const auto& position = precomputation.getReferenceEndEffectorPosition(i);
      const auto& velocity = precomputation.getReferenceEndEffectorLinearVelocity(i);
      const auto clearance = precomputation.getReferenceEndEffectorTerrainClearance(i);
      const auto& normal = precomputation.getSurfaceNormal(i);
      const auto& rotationMatrix = precomputation.getRotationWorldToTerrain(i);

      EXPECT_TRUE((truePositons[i] - position).norm() < eps);
      EXPECT_TRUE((trueVelocities[i] - velocity).norm() < eps);
      EXPECT_TRUE((trueNormals[i] - normal).norm() < eps);
      EXPECT_TRUE((trueNormals[i] - rotationMatrix.transpose().col(2)).norm() < eps);
      EXPECT_TRUE(std::abs(trueClearances[i] - clearance) < eps);
    }
    testTime += testDeltaTime;
  }
}