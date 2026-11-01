#include <gtest/gtest.h>

#include <legged_locomotion_mpc/trajectory_planners/JointTrajectoryPlanner.hpp>
#include <legged_locomotion_mpc/kinematics/PinocchioForwardEndEffectorKinematicsCppAd.hpp>


#include <floating_base_model/AccessHelperFunctions.hpp>
#include <floating_base_model/FloatingBaseModelPinocchioMapping.hpp>
#include <floating_base_model/QuaterionEulerTransforms.hpp>
#include <floating_base_model/FactoryFunctions.hpp>

#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include "../include/definitions.hpp"

using namespace legged_locomotion_mpc;
using namespace legged_locomotion_mpc::planners;
using namespace ocs2;
using namespace terrain_model;
using namespace floating_base_model;
using namespace multi_end_effector_kinematics;

const scalar_t endEffectorPositionTolerance = 1e-5;
const scalar_t velocityTolerance = 1e-3;

const scalar_t stateTolerance = 1e-5;
const scalar_t inputTolerance = 1e-5;

const size_t NUM_TEST = 100;
const size_t POSITION_CHANGE_CONST = 100;

// Distance from joint limits for random joint positions (bigger than position change)
const scalar_t JOINT_LIMIT_MARGIN = 0.05;

// Joint velocity scale and limit margin for trajectory (joints drift at most 0.2 rad)
const scalar_t TRAJECTORY_JOINT_VELOCITY = 0.2;
const scalar_t TRAJECTORY_JOINT_LIMIT_MARGIN = 0.25;

/**
 * Get random joint positions between joint limits shrinked by margin
 * (inverse kinematics solver does not start outside of joint limits)
 */
vector_t getRandomJointPositions(const vector_t& lowerLimits,
  const vector_t& upperLimits, scalar_t margin)
{
  const vector_t lower = lowerLimits.array() + margin;
  const vector_t upper = upperLimits.array() - margin;
  const vector_t random = 0.5 * (vector_t::Random(lowerLimits.size()).array() + 1.0);
  return lower + random.cwiseProduct(upper - lower);
}

TEST(JointTrajectoryPlannerTest, computeJointPositions) 
{
  std::string urdfPathName = meldogWithBaseLinkUrdfFile;
  std::string solverName = "NewtonRaphson";

  KinematicsModelSettings modelSettings;
  modelSettings.baseLinkName = baseLink;
  modelSettings.threeDofEndEffectorNames = meldog3DofContactNames;

  InverseSolverSettings solverSettings;
  solverSettings.tolerance = 1e-8;
  MultiEndEffectorKinematics kinematicsSolver(urdfPathName, modelSettings,
    solverSettings, solverName);

  const vector_t jointLowerLimits = kinematicsSolver.getPinocchioModel().lowerPositionLimit;
  const vector_t jointUpperLimits = kinematicsSolver.getPinocchioModel().upperPositionLimit;

  ocs2::PinocchioInterface interface = createPinocchioInterfaceFromUrdfFile(urdfPathName, baseLink);
  const FloatingBaseModelInfo modelInfo = createFloatingBaseModelInfo(interface, meldog3DofContactNames, meldog6DofContactNames);

  JointTrajectoryPlanner planner(modelInfo, std::move(kinematicsSolver));

  std::string modelName = "meldogForwardKinematics";
  PinocchioForwardEndEffectorKinematicsCppAd forwardKinematics(interface,
    modelInfo, modelName);

  for(size_t i = 0; i < NUM_TEST; ++i)
  {
    const vector6_t currentBasePose = vector6_t::Random();
    const vector_t currentJointPositions = getRandomJointPositions(jointLowerLimits,
      jointUpperLimits, JOINT_LIMIT_MARGIN);
    const vector_t jointPositionsTrue = currentJointPositions +
      vector_t::Random(ACTUATED_DOF_NUM) / POSITION_CHANGE_CONST;

    vector_t state = vector_t::Zero(modelInfo.stateDim);

    access_helper_functions::
      getBasePose(state, modelInfo) = currentBasePose;
    access_helper_functions::
      getJointPositions(state, modelInfo) = jointPositionsTrue;

    const std::vector<vector3_t> endEffectorPositions = 
      forwardKinematics.getPosition(state);

    const std::vector<vector3_t> emptyNormals;

    const vector_t calculatedJointPositions = planner.computeJointPositions(
      currentJointPositions, currentBasePose, endEffectorPositions, emptyNormals);

    /**
     * Calculated joint positions reach target end effector positions. Joint positions
     * are not compared, inverse kinematics has more solutions (e.g. near straight leg)
     */
    access_helper_functions::
      getJointPositions(state, modelInfo) = calculatedJointPositions;

    const std::vector<vector3_t> calculatedEndEffectorPositions =
      forwardKinematics.getPosition(state);

    for(size_t j = 0; j < endEffectorPositions.size(); ++j)
    {
      EXPECT_TRUE((calculatedEndEffectorPositions[j] - endEffectorPositions[j]).norm()
        < endEffectorPositionTolerance);
    }

    EXPECT_TRUE((calculatedJointPositions.array() >= jointLowerLimits.array()).all());
    EXPECT_TRUE((calculatedJointPositions.array() <= jointUpperLimits.array()).all());
  }
}

TEST(JointTrajectoryPlannerTest, computeJointVelocities) 
{
  std::string urdfPathName = meldogWithBaseLinkUrdfFile;
  std::string solverName = "NewtonRaphson";

  KinematicsModelSettings modelSettings;
  modelSettings.baseLinkName = baseLink;
  modelSettings.threeDofEndEffectorNames = meldog3DofContactNames;

  InverseSolverSettings solverSettings;
  solverSettings.tolerance = 1e-8;
  MultiEndEffectorKinematics kinematicsSolver(urdfPathName, modelSettings,
    solverSettings, solverName);

  const vector_t jointLowerLimits = kinematicsSolver.getPinocchioModel().lowerPositionLimit;
  const vector_t jointUpperLimits = kinematicsSolver.getPinocchioModel().upperPositionLimit;

  ocs2::PinocchioInterface interface = createPinocchioInterfaceFromUrdfFile(urdfPathName, baseLink);
  const FloatingBaseModelInfo modelInfo = createFloatingBaseModelInfo(interface, meldog3DofContactNames, meldog6DofContactNames);

  JointTrajectoryPlanner planner(modelInfo, std::move(kinematicsSolver));

  std::string modelName = "meldogForwardKinematics";
  PinocchioForwardEndEffectorKinematicsCppAd forwardKinematics(interface,
    modelInfo, modelName);

  for(size_t i = 0; i < NUM_TEST; ++i)
  {
    const vector6_t currentBasePose = vector6_t::Random();
    const vector6_t currentBaseVelocity = vector6_t::Random();
    const vector_t currentJointPositions = getRandomJointPositions(jointLowerLimits,
      jointUpperLimits, JOINT_LIMIT_MARGIN);
    const vector_t jointVelocitiesTrue = vector_t::Random(ACTUATED_DOF_NUM);

    vector_t state = vector_t::Zero(modelInfo.stateDim);
    vector_t input = vector_t::Zero(modelInfo.inputDim);

    access_helper_functions::
      getBasePose(state, modelInfo) = currentBasePose;
    access_helper_functions::
      getBaseVelocity(state, modelInfo) = currentBaseVelocity;
    access_helper_functions::
      getJointPositions(state, modelInfo) = currentJointPositions;
    

    access_helper_functions::
      getJointVelocities(input, modelInfo) = jointVelocitiesTrue;

    const std::vector<vector3_t> endEffectorVelocities =
      forwardKinematics.getLinearVelocity(state, input);

    // Previous velocities are zero, so test fails if solver does not return velocities
    const vector_t previousJointVelocities = vector_t::Zero(ACTUATED_DOF_NUM);

    const vector_t calculatedJointVelocities = planner.computeJointVelocities(
      currentJointPositions, currentBasePose, currentBaseVelocity, endEffectorVelocities,
      previousJointVelocities);

    EXPECT_TRUE((calculatedJointVelocities - jointVelocitiesTrue).norm() < velocityTolerance);
  }
}

TEST(JointTrajectoryPlannerTest, updateTrajectory) 
{
  std::string urdfPathName = meldogWithBaseLinkUrdfFile;
  std::string solverName = "NewtonRaphson";

  KinematicsModelSettings modelSettings;
  modelSettings.baseLinkName = baseLink;
  modelSettings.threeDofEndEffectorNames = meldog3DofContactNames;

  InverseSolverSettings solverSettings;
  solverSettings.tolerance = 1e-8;
  MultiEndEffectorKinematics kinematicsSolver(urdfPathName, modelSettings,
    solverSettings, solverName);

  const vector_t jointLowerLimits = kinematicsSolver.getPinocchioModel().lowerPositionLimit;
  const vector_t jointUpperLimits = kinematicsSolver.getPinocchioModel().upperPositionLimit;

  ocs2::PinocchioInterface interface = createPinocchioInterfaceFromUrdfFile(urdfPathName, baseLink);
  const FloatingBaseModelInfo modelInfo = createFloatingBaseModelInfo(interface, meldog3DofContactNames, meldog6DofContactNames);

  JointTrajectoryPlanner planner(modelInfo, std::move(kinematicsSolver));

  std::string modelName = "meldogForwardKinematics";
  PinocchioForwardEndEffectorKinematicsCppAd forwardKinematics(interface,
    modelInfo, modelName);

  const scalar_t initTime = 0.0;
  const scalar_t endTime = 1.0;
  const scalar_t deltaTime = 0.1;
  const size_t referenceSize = (endTime - initTime) / 0.1 + 1;

  TargetTrajectories trajectory;
  TargetTrajectories trajectoryTrue;

  trajectory.timeTrajectory.resize(referenceSize);
  trajectoryTrue.timeTrajectory.resize(referenceSize);

  vector_t state = vector_t::Zero(modelInfo.stateDim);
  vector_t input = vector_t::Zero(modelInfo.inputDim);

  trajectory.stateTrajectory.resize(referenceSize, state);
  trajectoryTrue.stateTrajectory.resize(referenceSize, state);

  trajectory.inputTrajectory.resize(referenceSize, input);
  trajectoryTrue.inputTrajectory.resize(referenceSize, input);


  using position_trajectories = std::vector<std::vector<vector3_t>>;
  using velocity_trajectories = std::vector<std::vector<vector3_t>>;

  position_trajectories endEffectorPositionsTrajectories;
  position_trajectories endEffectorVelocitiesTrajectories;

  std::vector<vector_t> jointPositionsTrue;
  std::vector<vector_t> jointVelocitiesTrue;

  vector6_t currentBasePose = vector6_t::Random();
  vector6_t currentBaseVelocity = vector6_t::Random();

  // Joints stay inside joint limits during whole trajectory
  vector_t currentJointPositions = getRandomJointPositions(jointLowerLimits,
    jointUpperLimits, TRAJECTORY_JOINT_LIMIT_MARGIN);
  vector_t currentJointVelocities = vector_t::Random(ACTUATED_DOF_NUM)
    * TRAJECTORY_JOINT_VELOCITY;

  state = vector_t::Zero(modelInfo.stateDim);
  input = vector_t::Zero(modelInfo.inputDim);

  access_helper_functions::
    getBasePose(state, modelInfo) = currentBasePose;
  access_helper_functions::
    getBaseVelocity(state, modelInfo) = currentBaseVelocity;
  access_helper_functions::
    getJointPositions(state, modelInfo) = currentJointPositions;
    
  access_helper_functions::
    getJointVelocities(input, modelInfo) = currentJointVelocities;
  
  trajectory.timeTrajectory[0] = initTime;
  trajectoryTrue.timeTrajectory[0] = initTime;

  trajectoryTrue.stateTrajectory[0] = state;
  trajectoryTrue.inputTrajectory[0] = input;

  // Give only data of base trajectory
  trajectory.stateTrajectory[0] = state;

  const std::vector<vector3_t> startEndEffectorPositions = 
    forwardKinematics.getPosition(state);
  endEffectorPositionsTrajectories.push_back(startEndEffectorPositions);

  const std::vector<vector3_t> startEndEffectorVelocities = 
    forwardKinematics.getLinearVelocity(state, input);
  endEffectorVelocitiesTrajectories.push_back(startEndEffectorVelocities);

  for(size_t i = 1; i < referenceSize; ++i)
  {
    currentBasePose += currentBaseVelocity * deltaTime;
    currentJointPositions += currentJointVelocities * deltaTime;

    currentBaseVelocity = vector6_t::Random();
    currentJointVelocities = vector_t::Random(ACTUATED_DOF_NUM) * TRAJECTORY_JOINT_VELOCITY;

    state = vector_t::Zero(modelInfo.stateDim);
    input = vector_t::Zero(modelInfo.inputDim);

    access_helper_functions::
      getBasePose(state, modelInfo) = currentBasePose;
    access_helper_functions::
      getBaseVelocity(state, modelInfo) = currentBaseVelocity;

    // Give only data of base trajectory
    trajectory.stateTrajectory[i] = state;
    trajectory.inputTrajectory[i] = input;
    
    access_helper_functions::
      getJointPositions(state, modelInfo) = currentJointPositions;
    
    access_helper_functions::
      getJointVelocities(input, modelInfo) = currentJointVelocities;

    trajectory.timeTrajectory[i] = initTime + i * deltaTime;
    trajectoryTrue.timeTrajectory[i] = initTime + i * deltaTime;

    trajectoryTrue.stateTrajectory[i] = state;
    trajectoryTrue.inputTrajectory[i] = input;

    const std::vector<vector3_t> endEffectorPositions = forwardKinematics.getPosition(state);
    endEffectorPositionsTrajectories.push_back(endEffectorPositions);

    const std::vector<vector3_t> endEffectorVelocities = 
      forwardKinematics.getLinearVelocity(state, input);
    endEffectorVelocitiesTrajectories.push_back(endEffectorVelocities);

  }

  SystemObservation initialObservation;
  initialObservation.state = vector_t::Zero(modelInfo.stateDim);
  initialObservation.input = vector_t::Zero(modelInfo.inputDim);

  auto& currentState = initialObservation.state;
  auto& currentInput = initialObservation.input;

  access_helper_functions::
    getBasePose(currentState, modelInfo) = 
    access_helper_functions::
    getBasePose(trajectoryTrue.stateTrajectory[0], modelInfo);

  access_helper_functions::
    getBaseVelocity(currentState, modelInfo) = 
    access_helper_functions::
    getBaseVelocity(trajectoryTrue.stateTrajectory[0], modelInfo);

  access_helper_functions::
    getJointPositions(currentState, modelInfo) = 
    access_helper_functions::
    getJointPositions(trajectoryTrue.stateTrajectory[0], modelInfo);

  access_helper_functions::
    getJointVelocities(currentInput, modelInfo) = 
    access_helper_functions::
    getJointVelocities(trajectoryTrue.inputTrajectory[0], modelInfo);

    const std::vector<std::vector<vector3_t>> emptyNormalTrajectories(
      endEffectorPositionsTrajectories.size());
  
  planner.updateTrajectory(initialObservation, trajectory, 
    endEffectorPositionsTrajectories, 
    endEffectorVelocitiesTrajectories, emptyNormalTrajectories);

  for(size_t i = 0; i < referenceSize; ++i)
  {
    EXPECT_TRUE(std::abs(trajectory.timeTrajectory[i] - 
      trajectoryTrue.timeTrajectory[i]) < 1e-9);
    
    EXPECT_TRUE((trajectory.stateTrajectory[i] - 
      trajectoryTrue.stateTrajectory[i]).norm() < stateTolerance);
      
    EXPECT_TRUE((trajectory.inputTrajectory[i] - 
      trajectoryTrue.inputTrajectory[i]).norm() < stateTolerance);
  }
}

TEST(JointTrajectoryPlannerTest, loaders) 
{
  const std::string kinematicsModelFilePath = meldogConfigFolder + "model_settings.info";

  const auto modelSettings = loadKinematicsModelSettings(kinematicsModelFilePath);

  EXPECT_TRUE(modelSettings.baseLinkName == "trunk_link");
  EXPECT_TRUE(modelSettings.threeDofEndEffectorNames == meldog3DofContactNames);
  EXPECT_TRUE(modelSettings.sixDofEndEffectorNames.size() == 0);

  const std::string solverFilePath = meldogConfigFolder + "inverse_solver_settings.info";

  const auto solverSettings = loadInverseSolverSettings(solverFilePath);

  const auto solverName = loadInverseSolverName(solverFilePath);

  EXPECT_TRUE(solverSettings.maxIterations        == 1000);
  EXPECT_TRUE(solverSettings.tolerance            == 0.1);
  EXPECT_TRUE(solverSettings.minimumStepSize      == 0.2);
  EXPECT_TRUE(solverSettings.dampingCoefficient   == 0.3);
  EXPECT_TRUE(solverSettings.stepCoefficient      == 0.4);
  EXPECT_TRUE(solverSettings.singularityThreshold == 0.5);

  EXPECT_TRUE(solverName == "NewtonRaphson");
}