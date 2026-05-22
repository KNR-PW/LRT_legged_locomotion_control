// #include "rclcpp/rclcpp.hpp"

#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_sqp/SqpMpc.h>
#include <ocs2_ddp/GaussNewtonDDP_MPC.h>

#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include <ocs2_oc/oc_data/PerformanceIndex.h>
#include <ocs2_oc/oc_data/LoopshapingPrimalSolution.h>

#include <terrain_model/core/TerrainPlane.hpp>
#include <terrain_model/planar_model/PlanarTerrainModel.hpp>

#include <floating_base_model/FactoryFunctions.hpp>

#include <legged_locomotion_mpc/robot_interface/LeggedLoopshapingInterface.hpp>

#include <legged_locomotion_mpc_ros2/path_management/package_path.h>

#include <legged_locomotion_mpc_ros2/visualization/LeggedVisualizer.hpp>

#include "../../../include/legged_locomotion_mpc_ros2/demos/melman_demo/melman_definitions.hpp"

using namespace ocs2;
using namespace terrain_model;
using namespace floating_base_model;
using namespace legged_locomotion_mpc;
using namespace legged_locomotion_mpc::utils;
using namespace legged_locomotion_mpc::locomotion;
using namespace legged_locomotion_mpc::planners;
using namespace legged_locomotion_mpc_ros2;
using namespace legged_locomotion_mpc_ros2::melman_definitions;

int main(int argc, char* argv[]) 
{
  const std::string configFilePath = configDirectory;
  const std::string taskFilePath = configFilePath + "task.info";
  const std::string modelFilePath = configFilePath + "legged_model.info";
  const std::string urdfFilePath = urdfFile;
  const std::string loopshapingFilePath = configFilePath + "loopshaping.info";

  const scalar_t initTime = 0.0;

  const vector3_t terrainEulerZyx{0.0, 0.0, 0.0};
  const matrix3_t terrainRotation = getRotationMatrixFromZyxEulerAngles(terrainEulerZyx); 

  TerrainPlane terrainPlane(vector3_t::Zero(), terrainRotation.transpose());

  std::unique_ptr<TerrainModel> terrainModel = 
    std::make_unique<PlanarTerrainModel>(terrainPlane);

  ocs2::PinocchioInterface interface = createPinocchioInterfaceFromUrdfFile(urdfFilePath, baseLink);
  const FloatingBaseModelInfo modelInfo = createFloatingBaseModelInfo(interface, threeDofContactNames, sixDofContactNames);

  const vector2_t initPositionXY{0.0, 0.0};

  vector3_t initialBasePosition(initPositionXY.x(), initPositionXY.y(),
    terrainModel->getSmoothedPositon(initPositionXY).z());

  BaseTrajectoryPlanner::StaticSettings baseSettings = 
    loadBasePlannerStaticSettings(modelFilePath);
  
  initialBasePosition.z() += baseSettings.initialBaseHeight / terrainPlane.getSurfaceNormalInWorld().z();

  vector_t initialState = vector_t::Zero(STATE_DIM);
  initialState.block<3,1>(6, 0) = initialBasePosition;
  initialState.block<3,1>(9, 0) = terrainEulerZyx;
  access_helper_functions::getJointPositions(initialState, modelInfo) << 0.78539816339, 1e-6, 2 * 0.78539816339, 1e-6, 1e-6, 0.4, -0.8, 0.4, 1e-6, 0.78539816339, 1e-6, 2 * 0.78539816339, 1e-6, 1e-6, 0.4, -0.8, 0.4, 1e-6;
  // access_helper_functions::getJointPositions(initialState, modelInfo) << 0, -0.785398163, 1.570796326, 0, -0.785398163, 1.570796326, 0, -0.785398163, 1.570796326, 0, -0.785398163, 1.570796326;

  LeggedLoopshapingInterface loopShapingInterface = makeLeggedLoopshapingInterface(initTime, 
    initialState, std::move(terrainModel), taskFilePath, modelFilePath, urdfFilePath, 
    loopshapingFilePath);

  LeggedInterface& leggedInterface = loopShapingInterface.getLeggedInterface();

  auto& referenceManager = leggedInterface.getLeggedReferenceManager();

  /* STANDING TROT */
  
  GaitDynamicParameters firstDynamicParams;
  firstDynamicParams.steppingFrequency = 0.5 * 1.0 / 0.7;
  firstDynamicParams.swingRatio = 3.0 / 7.0;

  const scalar_t firstOffset = 3.5 / 7.0;
  
  firstDynamicParams.phaseOffsets = {-firstOffset};

  BaseTrajectoryPlanner::BaseReferenceCommand firstCommand;
  firstCommand.baseHeadingVelocity = 0.025;
  firstCommand.baseLateralVelocity = 0.0;
  firstCommand.baseVerticalVelocity = 0.0;
  firstCommand.yawRate = 1 * 0.0;

  BaseTrajectoryPlanner::BaseReferenceCommand secondCommand;
  secondCommand.baseHeadingVelocity = 0.1;
  secondCommand.baseLateralVelocity = -0.0;
  secondCommand.baseVerticalVelocity = 0.0;
  secondCommand.yawRate = -1 * 0.0;

  const scalar_t firstGaitTime = 1.0;
  bool firstChange = true;
  const scalar_t secondGaitTime = 8.0;
  bool secondChange = true;

  const scalar_t firstMoveTime = 2.0;
  const scalar_t secondMoveTime = 20.0;
  const scalar_t endTime = 15.0;

  // DDP

  const auto mpcSettings = leggedInterface.mpcSettings();
  const auto sqpSettings = leggedInterface.sqpSettings();
  const auto ddpSettings = leggedInterface.ddpSettings();

  const auto& optimalProblem = loopShapingInterface.getOptimalControlProblem();
  const auto& initializer = loopShapingInterface.getInitializer();

  auto& rollout = loopShapingInterface.getRollout();

  auto& weightCompenator = leggedInterface.weightCompensator();

  // std::unique_ptr<MPC_BASE> mpcPtr = std::make_unique<GaussNewtonDDP_MPC>(mpcSettings, 
  //   ddpSettings, rollout, optimalProblem, initializer);

  std::unique_ptr<MPC_BASE> mpcPtr = std::make_unique<SqpMpc>(mpcSettings, sqpSettings, 
    optimalProblem, initializer);

  mpcPtr->getSolverPtr()->setReferenceManager(loopShapingInterface.getReferenceManagerPtr());

  std::unique_ptr<RolloutBase> rolloutPtr(rollout.clone());

  MPC_MRT_Interface mpcInterface(*mpcPtr);

  const size_t standingMode = ((0x01 << (NUM_SIX_DOF_CONTACTS)) - 1);
  const contact_flags_t standingFlags(standingMode);

  /// ====== Execute the scenario ========
  SystemObservation observation;
  observation.time = initTime;
  observation.state = loopShapingInterface.getInitialState();
  observation.input = vector_t::Zero(INPUT_DIM);

  // Wait for the first policy
  mpcInterface.setCurrentObservation(observation);

  std::cerr << "Initialization!" << std::endl;
  while (!mpcInterface.initialPolicyReceived()) 
  {
    mpcInterface.advanceMpc();
  }
  std::cerr << "Init rollout" << std::endl;
  mpcInterface.initRollout(&leggedInterface.getRollout());

  std::cerr << "Main loop!" << std::endl;

  // run MPC till final time
  TargetTrajectories optimalTrajectory;
  ModeSchedule modeSchedule;
  modeSchedule.clear();
  std::vector<PerformanceIndex> performances;

  std::vector<ocs2::SystemObservation> observations;
  std::vector<contact_flags_t> contactFlags;

  std::vector<ocs2::scalar_array_t> optimizedTimeTrajectories;
  std::vector<ocs2::vector_array_t> optimizedStateTrajectories;

  while(observation.time < endTime) 
  {
    std::cout << "Time: " << observation.time << "\n";
    observations.push_back(observation);
    try 
    {
      // run MPC at current observation
      mpcInterface.setCurrentObservation(observation);
     
      if(observation.time > firstGaitTime && firstChange)
      {
        firstChange = false;
        referenceManager.updateGaitParemeters(firstDynamicParams);
      }

      if(observation.time > firstMoveTime)
      {
        referenceManager.updateCommand(firstCommand);
      }

      if(observation.time > secondMoveTime)
      {
        referenceManager.updateCommand(secondCommand);
      }

      const auto& lastTrajectoryState = referenceManager.getTargetTrajectories().stateTrajectory.back();

      mpcInterface.advanceMpc();
      mpcInterface.updatePolicy();

      performances.push_back(mpcInterface.getPerformanceIndices());

      // Evaluate the optimized solution - change to optimal controller
      vector_t tmp;
      mpcInterface.evaluatePolicy(observation.time, observation.state, tmp,
        observation.input, observation.mode);

      
      observation.input = LinearInterpolation::interpolate(
        observation.time, mpcInterface.getPolicy().timeTrajectory_,
        mpcInterface.getPolicy().inputTrajectory_);

      optimizedTimeTrajectories.push_back(mpcInterface.getPolicy().timeTrajectory_);
      optimizedStateTrajectories.push_back(mpcInterface.getPolicy().stateTrajectory_);

      optimalTrajectory.timeTrajectory.push_back(observation.time);
      optimalTrajectory.stateTrajectory.push_back(observation.state);
      optimalTrajectory.inputTrajectory.push_back(observation.input);


      if(modeSchedule.modeSequence.empty()) 
      {
        modeSchedule.modeSequence.push_back(observation.mode);
      } 
      else if(modeSchedule.modeSequence.back() != observation.mode) 
      {
        modeSchedule.modeSequence.push_back(observation.mode);
        modeSchedule.eventTimes.push_back(observation.time);
      }

      // perform a rollout
      scalar_array_t timeTrajectory;
      size_array_t postEventIndicesStock;
      vector_array_t stateTrajectory, inputTrajectory;
      const scalar_t finalTime = observation.time +  1.0 / mpcSettings.mpcDesiredFrequency_;
      auto modeschedule = mpcInterface.getPolicy().modeSchedule_;
      rolloutPtr->run(observation.time, observation.state, finalTime,
                   mpcInterface.getPolicy().controllerPtr_.get(), modeschedule,
                   timeTrajectory, postEventIndicesStock, stateTrajectory,
                   inputTrajectory);

      observation.time = finalTime;
      observation.state = stateTrajectory.back();
      observation.input = inputTrajectory.back();

      const auto targetState = referenceManager.getTargetTrajectories().getDesiredState(finalTime);
      const auto targetInput = referenceManager.getTargetTrajectories().getDesiredInput(finalTime);
    } 
    catch (std::exception& e) 
    {
      std::cout << "MPC failed\n";
      std::cout << e.what() << "\n";
      break;
    }
  }

  rclcpp::init(argc, argv);

  const auto visualizerSettings = LeggedVisualizer::Settings();
  const auto modelSettings = leggedInterface.modelSettings();
  const auto& forwardKinematics = leggedInterface.forwardKinematics();
  const auto& torqueApproximator = leggedInterface.torqueApproximator();
  const auto& robotModel = leggedInterface.pinocchioInterface().getModel();

  const auto leggedVisualizer = std::make_shared<LeggedVisualizer>(visualizerSettings, 
    modelSettings, modelInfo, robotModel, forwardKinematics, torqueApproximator);

  while(true)
  {
    scalar_t visualizationTime = initTime;
    size_t visualizationIndex = 0;
    while(visualizationTime < observations.back().time)
    {
      const auto currentTimeStamp = leggedVisualizer->now();
      leggedVisualizer->publishObservation(currentTimeStamp, observations[visualizationIndex]);
      if(visualizationIndex < optimizedTimeTrajectories.size())
      {
        leggedVisualizer->publishOptimizedTrajectory(currentTimeStamp, 
          optimizedTimeTrajectories[visualizationIndex], 
          optimizedStateTrajectories[visualizationIndex]);
      }
      visualizationIndex++;
      if(visualizationIndex < observations.size())
      {
        const auto duration = observations[visualizationIndex].time - visualizationTime;
        const auto durationSeconds = std::chrono::duration<scalar_t>(duration);
        const auto durationNanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(
          durationSeconds);
        rclcpp::sleep_for(durationNanoseconds);
        visualizationTime = observations[visualizationIndex].time;
      }
    }
  }
  
  rclcpp::shutdown();
  return 0;
}