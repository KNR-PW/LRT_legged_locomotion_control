#include <benchmark/benchmark.h>

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

static void LeggedReferenceManager_PRESOLVE(benchmark::State & state)
{
  const scalar_t defTime = 0.0;
  const scalar_t initTime = 0.0;
  const scalar_t finalTime = 1.0;
  const scalar_t deltaTime = 0.01;

  /* STANDING TROT */
  scalar_t startingPhase = 3.5 / 7.0;

  GaitStaticParameters staticParams;
  staticParams.endEffectorNumber = 4;

  /* FLYING TROT */
  GaitDynamicParameters dynamicParams;
  dynamicParams.swingRatio =  0.33 / 0.6;
  dynamicParams.steppingFrequency = 1.0 / 0.6;
  scalar_t currentPhase = dynamicParams.swingRatio;

  dynamicParams.phaseOffsets = {-currentPhase + 0.03 / 0.6, -currentPhase + 0.03 / 0.6, 0};

  auto modeSequenceTemplate = getDynamicModeSequenceTemplate(startingPhase,
    finalTime, staticParams, dynamicParams);

  const scalar_t slope = 0.0;
  const vector3_t terrainEulerZyx{0.0, slope, 0.0};
  const matrix3_t terrainRotation = getRotationMatrixFromZyxEulerAngles(terrainEulerZyx); 

  TerrainPlane slopyTerrain(vector3_t::Random(), terrainRotation.transpose());

  PlanarTerrainModel terrainModel(slopyTerrain);

  BaseTrajectoryPlanner::StaticSettings staticSettings;
  staticSettings.deltaTime = deltaTime;
  staticSettings.initialBaseHeight = 0.4;
  staticSettings.minimumBaseHeight = 0.1;
  staticSettings.maximumBaseHeight = 5.0;
  staticSettings.nominalBaseWidthHeading = 0.2;
  staticSettings.nominalBaseWidthLateral = 0.2;

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

  GaitPlanner gaitPlanner(staticParams, dynamicParams, modeSequenceTemplate, 
    startingPhase, defTime);

  BaseTrajectoryPlanner basePlanner(modelInfo, staticSettings);

  SwingTrajectoryPlanner swingPlanner(modelInfo, swingStaticSettings, 
    swingDynamicSettings, forwardKinematics);

  JointTrajectoryPlanner jointPlanner(modelInfo, std::move(kinematicsSolver));

  ContactForceWrenchTrajectoryPlanner forcePlanner(modelInfo);
  
  LeggedReferenceManager::Settings managerSettings;

  LeggedReferenceManager manager(modelInfo, managerSettings, gaitPlanner, swingPlanner, basePlanner, 
    jointPlanner, forcePlanner);

  const auto modeSchedule = gaitPlanner.getModeSchedule(initTime, finalTime);

  const vector3_t initPosition = vector3_t::Random();

  const vector3_t planeLocalEulerZyx = vector3_t{std::rand() / scalar_t(RAND_MAX);, 0.0, 
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
    terrainModel.getSmoothedPosition(initPositionXY).z());
  
  initialBasePosition.z() += staticSettings.initialBaseHeight / slopyTerrain.getSurfaceNormalInWorld().z();
  
  state_vector_t initialState = state_vector_t::Zero(12 + modelInfo.actuatedDofNum * 2);
  initialState.block<3,1>(6, 0) = initialBasePosition;
  initialState.block<3,1>(9, 0) = initEulerZyx;

  legged_locomotion_mpc::access_helper_functions::getJointPositions(initialState, modelInfo) << 0, -0.52359877559, 1.0471975512, 0, -0.52359877559, 1.0471975512, 0, -0.52359877559, 1.0471975512, 0, -0.52359877559, 1.0471975512;
  
  contact_flags_t contactFlag = modeSchedule.modeAtTime(initTime);

  std::unique_ptr<TerrainModel> terrainModelPtr = 
    std::make_unique<PlanarTerrainModel>(slopyTerrain);

  manager.initialize(initTime, finalTime, initialState, contactFlag, 
    std::move(dynamicParams), std::move(terrainModelPtr));

   for (auto _ : state) {
    manager.preSolverRun(initTime, finalTime, initialState);
  }
}

BENCHMARK(LeggedReferenceManager_PRESOLVE);