#include <legged_locomotion_mpc/reference_manager/LeggedReferenceManager.hpp>

#include <limits>

#include <ocs2_core/misc/LinearInterpolation.h>

#include <legged_locomotion_mpc/common/Utils.hpp>

namespace legged_locomotion_mpc
{

  using namespace ocs2;
  using namespace locomotion;
  using namespace planners;
  using namespace terrain_model;
  using namespace floating_base_model;
  using EndEffectorTrajectories = legged_locomotion_mpc::locomotion::SwingTrajectoryPlanner::EndEffectorTrajectories;
  using EndEffectorTrajectoriesPoint = legged_locomotion_mpc::locomotion::SwingTrajectoryPlanner::EndEffectorTrajectoriesPoint;

  LeggedReferenceManager::LeggedReferenceManager(
    FloatingBaseModelInfo modelInfo,
    LeggedReferenceManager::Settings settings,
    GaitPlanner&& gaitPlanner,
    SwingTrajectoryPlanner&& swingPlanner,
    BaseTrajectoryPlanner&& basePlanner,
    JointTrajectoryPlanner&& jointPlanner,
    ContactForceWrenchTrajectoryPlanner&& forcePlanner):
      ReferenceManager(TargetTrajectories(), ModeSchedule()),
      modelInfo_(std::move(modelInfo)),
      settings_(std::move(settings)),
      gaitPlanner_(std::move(gaitPlanner)), 
      swingPlanner_(std::move(swingPlanner)),
      basePlanner_(std::move(basePlanner)), 
      jointPlanner_(std::move(jointPlanner)),
      forcePlanner_(std::move(forcePlanner)),
      currentContactFlags_(contact_flags_t()),
      currentGaitParameters_(GaitDynamicParameters()),
      currentSwingParameters_(SwingTrajectoryPlanner::DynamicSettings()),
      currentCommand_(BaseTrajectoryPlanner::BaseReferenceCommand()),
      bufferedTerrainModel_(nullptr),
      referenceTrajectories_(EndEffectorTrajectories()),
      footConstraintTrajectories_(FootTangentialConstraintTrajectories()) {}

  void LeggedReferenceManager::initialize(scalar_t initTime, scalar_t finalTime, 
    const SystemObservation& currentObservation, 
    const contact_flags_t& currentContactFlags,
    const GaitDynamicParameters& currentGaitParameters,
    const SwingTrajectoryPlanner::DynamicSettings& currentSwingParameters,
    std::unique_ptr<terrain_model::TerrainModel> currentTerrainModel)
  {
    // // Reset base orientation to be the parallel to terrain 
    auto stampedCurrentObservation = currentObservation;
    // auto baseEulerZyx = access_helper_functions::getBaseOrientationZyx(
    //   stampedCurrentObservation.state, modelInfo_);

    // const vector3_t currentBasePosition = access_helper_functions::getBasePosition(
    //   stampedCurrentObservation.state, modelInfo_);

    // const TerrainPlane currentPlane = currentTerrainModel->getLocalTerrainAtPositionInWorldAlongGravity(
    //   currentBasePosition);

    // const matrix3_t terrainToWorldRotation = currentPlane.getOrientationToTerrain().transpose();
    // const auto terrainEuler = quaterion_euler_transforms::getEulerAnglesFromRotationMatrix(
    //   terrainToWorldRotation);
    // baseEulerZyx(1) = terrainEuler(1);
    // baseEulerZyx(2) = terrainEuler(2);

    updateContactFlags(currentContactFlags);
    updateGaitParemeters(currentGaitParameters);
    updateSwingParameters(currentSwingParameters);

    // Setup initial target trajectory with stampedCurrentObservation
    TargetTrajectories initTargetTrajectories;
    initTargetTrajectories.timeTrajectory.push_back(initTime);
    initTargetTrajectories.stateTrajectory.push_back(stampedCurrentObservation.state);
    initTargetTrajectories.inputTrajectory.push_back(stampedCurrentObservation.input);
    this->setTargetTrajectories(initTargetTrajectories);
    
    // Set initial target trajectory
    ReferenceManager::preSolverRun(initTime, finalTime, 
      vector_t::Zero(modelInfo_.stateDim));

    // Get copy of current terrain for getter, buffered value might be changed in parallel task
    currentTerrainModel_ = std::unique_ptr<TerrainModel>(currentTerrainModel->clone());

    updateTerrainModel(std::move(currentTerrainModel));
    
    if(settings_.threaded)
    {
      newTrajectories_ = std::async(std::launch::async, [this, initTime, finalTime]()
      {return generateNewTargetTrajectories(initTime, finalTime);});
      newTrajectories_.wait();
    }
    else
    {
      generateNewTargetTrajectories(initTime, finalTime);
    }

    // Get reference trajectories and constraints
    referenceTrajectories_.updateFromBuffer();
    footConstraintTrajectories_.updateFromBuffer();

    // Get initialized target trajectory and mode schedule 
    ReferenceManager::preSolverRun(initTime, finalTime, 
      vector_t::Zero(modelInfo_.stateDim));
  }

  const contact_flags_t LeggedReferenceManager::getContactFlags(scalar_t time) const
  {
    const size_t index = utils::findIndexInTimeArray(getModeSchedule().eventTimes, time);
    return modeNumber2ContactFlags(getModeSchedule().modeSequence[index]);
  }

  const TerrainModel& LeggedReferenceManager::getTerrainModel() const
  {
    return *currentTerrainModel_.get();
  }

  EndEffectorTrajectoriesPoint LeggedReferenceManager::getEndEffectorTrajectoryPoint(
    scalar_t time) const
  {
    const auto& times = getTargetTrajectories().timeTrajectory;
    const auto indexAlpha = LinearInterpolation::timeSegment(time, times);
    size_t index = indexAlpha.first;
    const scalar_t alpha = indexAlpha.second;
    const scalar_t one_minus_alpha = 1.0 - alpha;

    // Get previous time index if value is between (times[index - 1], times[index])
    if(index != 0 && (times[index] - time) > std::numeric_limits<scalar_t>::min())
    {
      index -= 1;
    }

    const size_t numEndEffectors = modelInfo_.numThreeDofContacts + 
      modelInfo_.numSixDofContacts;

    EndEffectorTrajectoriesPoint point;

    point.positions.reserve(numEndEffectors);
    point.velocities.reserve(numEndEffectors);
    point.clearances.reserve(numEndEffectors);

    const EndEffectorTrajectories& referenceTrajectories = referenceTrajectories_.get();
    
    const auto& lhsPositions = referenceTrajectories.positions[index];
    const auto& rhsPositions = referenceTrajectories.positions[index + 1];

    const auto& lhsVelocities = referenceTrajectories.velocities[index];
    const auto& rhsVelocities = referenceTrajectories.velocities[index + 1];

    const auto& lhsClearances = referenceTrajectories.clearances[index];
    const auto& rhsClearances = referenceTrajectories.clearances[index + 1];

    const auto& lhsNormals = referenceTrajectories.surfaceNormals[index];
    const auto& rhsNormals = referenceTrajectories.surfaceNormals[index + 1];
    
    for(size_t i = 0; i < numEndEffectors; ++i)
    {
      const vector3_t& lhsPosition = lhsPositions[i];
      const vector3_t& rhsPosition = rhsPositions[i];

      const vector3_t& lhsVelocity = lhsVelocities[i];
      const vector3_t& rhsVelocity = rhsVelocities[i];

      const scalar_t lhsClearance = lhsClearances[i];
      const scalar_t rhsClearance = rhsClearances[i];

      const vector3_t& lhsNormal = lhsNormals[i];
      const vector3_t& rhsNormal = rhsNormals[i];

      point.positions.emplace_back(alpha * lhsPosition + one_minus_alpha * rhsPosition);
      point.velocities.emplace_back(alpha * lhsVelocity + one_minus_alpha * rhsVelocity);
      point.clearances.emplace_back(alpha * lhsClearance + one_minus_alpha * rhsClearance);
      point.surfaceNormals.emplace_back(alpha * lhsNormal + one_minus_alpha * rhsNormal);
    }
    return point;
  }

  const std::vector<FootTangentialConstraintMatrix>& LeggedReferenceManager::getEndEffectorConstraintMatrixes(
    scalar_t time) const
  {
    const auto& footTangentialConstraintMatrixes = footConstraintTrajectories_.get();
    const auto& times = footTangentialConstraintMatrixes.times;
    const size_t index = utils::findIndexInTimeArray(times, time);
    if(index > times.size()) return footTangentialConstraintMatrixes.constraints.back();
    return footTangentialConstraintMatrixes.constraints[index];
  }

  void LeggedReferenceManager::preSolverRun(scalar_t initTime, scalar_t finalTime, 
    const vector_t& initState)
  {
    if(settings_.threaded)
    {
      if(newTrajectories_.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
      {
        return;
      }

      ReferenceManager::preSolverRun(initTime, finalTime, initState);

      // Get reference trajectories and constraints
      referenceTrajectories_.updateFromBuffer();
      footConstraintTrajectories_.updateFromBuffer();

      // Get copy of current terrain for getter, active value might be changed in parallel task
      currentTerrainModel_ = std::unique_ptr<TerrainModel>(bufferedTerrainModel_.get().clone());

      newTrajectories_ = std::async(std::launch::async, [this, initTime, finalTime]()
        {return generateNewTargetTrajectories(initTime, finalTime);});
    }
    else
    {
      generateNewTargetTrajectories(initTime, finalTime);

      ReferenceManager::preSolverRun(initTime, finalTime, initState);

      // Get reference trajectories and constraints
      referenceTrajectories_.updateFromBuffer();
      footConstraintTrajectories_.updateFromBuffer();

       // Get copy of current terrain for getter, active value might be changed in parallel task
      currentTerrainModel_ = std::unique_ptr<TerrainModel>(bufferedTerrainModel_.get().clone());
    }
  }

  void LeggedReferenceManager::generateNewTargetTrajectories(
    scalar_t initTime, scalar_t finalTime)
  {
    // Get current reference observation
    const auto& currentTargetTrajectory = getTargetTrajectories();

    SystemObservation currentObservation;
    currentObservation.time = initTime;
    currentObservation.state = currentTargetTrajectory.getDesiredState(initTime);
    currentObservation.input = currentTargetTrajectory.getDesiredInput(initTime);

    // no new gait parameters -> no update
    if(currentGaitParameters_.updateFromBuffer())
    {
      const GaitDynamicParameters& currentGaitParameters = currentGaitParameters_.get();
      gaitPlanner_.updateDynamicParameters(initTime + settings_.gaitUpdateOffset, currentGaitParameters);
    }

    // No new swing parameters -> no update
    if(currentSwingParameters_.updateFromBuffer())
    {
      const SwingTrajectoryPlanner::DynamicSettings& currentSwingParameters = currentSwingParameters_.get();
      swingPlanner_.updateDynamicSettings(currentSwingParameters);
    }

    // No new contact flag -> no update
    if(currentContactFlags_.updateFromBuffer())
    {
      const contact_flags_t& currentContactFlags = currentContactFlags_.get();
      gaitPlanner_.updateCurrentContacts(initTime, currentContactFlags);
    }

    // No new terrain model -> no update
    if(bufferedTerrainModel_.updateFromBuffer())
    {
      const TerrainModel& currentTerrainModel = bufferedTerrainModel_.get();

      basePlanner_.updateTerrain(currentTerrainModel);
    
      swingPlanner_.updateTerrain(currentTerrainModel);
    }
    
    // No new command -> stay in place
    BaseTrajectoryPlanner::BaseReferenceCommand currentCommand;
    if(currentCommand_.updateFromBuffer())
    {
      currentCommand = currentCommand_.get();
    }
    else
    {
      currentCommand.baseHeadingVelocity = 0.0;
      currentCommand.baseLateralVelocity = 0.0;
      currentCommand.baseVerticalVelocity = 0.0;
      currentCommand.yawRate = 0.0;
    }

    TargetTrajectories newTrajectory;

    const ModeSchedule newModeSchedule = gaitPlanner_.getModeSchedule(initTime, 
      finalTime);

    basePlanner_.updateTargetTrajectory(initTime, finalTime, currentCommand, 
      currentObservation, newTrajectory);
    
    // Update current base height in swing trajectory planner
    const scalar_t currentBaseHeight = basePlanner_.getBaseHeight();
    swingPlanner_.updateInvertedPendulumHeight(currentBaseHeight);

    swingPlanner_.updateSwingMotions(initTime, finalTime, currentObservation, 
      newTrajectory, newModeSchedule);

    EndEffectorTrajectories endEffectorTrajectories = 
      swingPlanner_.getEndEffectorTrajectories(newTrajectory.timeTrajectory);
    
    jointPlanner_.updateTrajectory(currentObservation, newTrajectory, 
      endEffectorTrajectories.positions, endEffectorTrajectories.velocities, 
      endEffectorTrajectories.surfaceNormals);

    // If trajectory was subsampled, get new reference for end effectors
    if(settings_.maximumReferenceSampleInterval < basePlanner_.getStaticSettings().deltaTime)
    {
      TargetTrajectories subsampledTrajectory = utils::subsampleReferenceTrajectory(
        newTrajectory, initTime, finalTime, settings_.maximumReferenceSampleInterval);

      forcePlanner_.updateTargetTrajectory(newModeSchedule, subsampledTrajectory);

      endEffectorTrajectories = swingPlanner_.getEndEffectorTrajectories(
        subsampledTrajectory.timeTrajectory);

      setTargetTrajectories(std::move(subsampledTrajectory));
    }
    else
    {
      forcePlanner_.updateTargetTrajectory(newModeSchedule, newTrajectory);

      setTargetTrajectories(std::move(newTrajectory));
    }

    const FootTangentialConstraintTrajectories footConstraintTrajectories = 
        swingPlanner_.getFootTangentialConstraintTrajectories();
    footConstraintTrajectories_.setBuffer(std::move(footConstraintTrajectories));

    referenceTrajectories_.setBuffer(std::move(endEffectorTrajectories));
    setModeSchedule(std::move(newModeSchedule));
  }

  void LeggedReferenceManager::updateContactFlags(
    const contact_flags_t& currentContactFlags)
  {
    currentContactFlags_.setBuffer(currentContactFlags);
  }

  void LeggedReferenceManager::updateGaitParemeters(
    const GaitDynamicParameters& currentGaitParameters)
  {
    currentGaitParameters_.setBuffer(currentGaitParameters);
  }

  void LeggedReferenceManager::updateSwingParameters(
    const SwingTrajectoryPlanner::DynamicSettings& currentSwingParameters)
  {
    currentSwingParameters_.setBuffer(currentSwingParameters);
  }
  
  void LeggedReferenceManager::updateTerrainModel(
    std::unique_ptr<TerrainModel> currentTerrainModel)
  {
    bufferedTerrainModel_.setBuffer(std::move(currentTerrainModel));
  }

  void LeggedReferenceManager::updateCommand(
      const BaseTrajectoryPlanner::BaseReferenceCommand& currentCommand)
  {
    currentCommand_.setBuffer(currentCommand);
  }

  LeggedReferenceManager::~LeggedReferenceManager()
  {
    if(settings_.threaded) newTrajectories_.wait();
  }

  LeggedReferenceManager::Settings loadLeggedReferenceManagerSettings(
    const std::string& filename, const std::string& fieldName, bool verbose)
  {
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(filename, pt);

    if(verbose) 
    {
      std::cerr << "\n #### Legged Locomotion MPC Reference Manager Settings:";
      std::cerr << "\n #### =============================================================================\n";
    }

    LeggedReferenceManager::Settings settings;

    loadData::loadPtreeValue(pt, settings.maximumReferenceSampleInterval, 
      fieldName + ".maximumReferenceSampleInterval", verbose);
    if(settings.maximumReferenceSampleInterval < 0.0)
    {
      throw std::invalid_argument("[LeggedReferenceManager]: Maximum reference sample interval smaller than 0!");
    }

    loadData::loadPtreeValue(pt, settings.gaitUpdateOffset, 
      fieldName + ".gaitUpdateOffset", verbose);
    if(settings.gaitUpdateOffset < 0.0)
    {
      throw std::invalid_argument("[LeggedReferenceManager]: Gait update offset smaller than 0!");
    }

    loadData::loadPtreeValue(pt, settings.threaded, 
      fieldName + ".threaded", verbose);

    if(verbose) 
    {
      std::cerr << " #### ==================================================" << std::endl;
    }

    return settings;
  }
} // namespace legged_locomotion_mpc
