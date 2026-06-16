#include <legged_locomotion_mpc/robot_interface/LeggedInterface.hpp>

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_oc/rollout/TimeTriggeredRollout.h>
#include <ocs2_ddp/ContinuousTimeLqr.h>

#include <floating_base_model/FactoryFunctions.hpp>

#include <legged_locomotion_mpc/cost/JointTorqueCost.hpp>
#include <legged_locomotion_mpc/cost/TerminalTrackingCost.hpp>
#include <legged_locomotion_mpc/cost/TrajectoryTrackingCost.hpp>

#include <legged_locomotion_mpc/constraint/NormalVelocityConstraint.hpp>
#include <legged_locomotion_mpc/constraint/Zero3DofVelocityConstraint.hpp>
#include <legged_locomotion_mpc/constraint/Zero6DofVelocityConstraint.hpp>
#include <legged_locomotion_mpc/constraint/ZeroForceConstraint.hpp>
#include <legged_locomotion_mpc/constraint/ZeroWrenchConstraint.hpp>

#include <legged_locomotion_mpc/soft_constraint/ForceFrictionConeSoftConstraint.hpp>
#include <legged_locomotion_mpc/soft_constraint/WrenchFrictionConeSoftConstraint.hpp>
#include <legged_locomotion_mpc/soft_constraint/EndEffectorPlacementSoftConstraint.hpp>
#include <legged_locomotion_mpc/soft_constraint/JointLimitsSoftConstraint.hpp>
#include <legged_locomotion_mpc/soft_constraint/JointTorqueLimitsSoftConstraint.hpp>
#include <legged_locomotion_mpc/soft_constraint/SelfCollisionAvoidanceSoftConstraint.hpp>
#include <legged_locomotion_mpc/soft_constraint/TerrainAvoidanceSoftConstraint.hpp>
#include <legged_locomotion_mpc/soft_constraint/NormalOrientationSoftConstraint.hpp>

#include <legged_locomotion_mpc/locomotion/GaitPlanner.hpp>
#include <legged_locomotion_mpc/locomotion/SwingTrajectoryPlanner.hpp>

#include <legged_locomotion_mpc/trajectory_planners/BaseTrajectoryPlanner.hpp>
#include <legged_locomotion_mpc/trajectory_planners/ContactForceWrenchTrajectoryPlanner.hpp>
#include <legged_locomotion_mpc/trajectory_planners/JointTrajectoryPlanner.hpp>

#include <legged_locomotion_mpc/dynamics/LeggedDynamicsAD.hpp>

namespace legged_locomotion_mpc
{
  using namespace ocs2;
  using namespace floating_base_model;
  using namespace terrain_model;
  using namespace planners;
  using namespace collision;
  using namespace locomotion;
  using namespace cost;
  using namespace utils;
  using namespace synchronization;
  using namespace multi_end_effector_kinematics;

  LeggedInterface::LeggedInterface(scalar_t initTime, const vector_t& currentState, 
    std::unique_ptr<TerrainModel> currentTerrainModel,
    const std::string& taskFile, const std::string& modelFile,
    const std::string& urdfFile)
  {
    initialState_ = currentState;

    // Check that task file exists
    const boost::filesystem::path taskFilePath(taskFile);
    if (exists(taskFilePath)) 
    {
      std::cerr << "[LeggedInterface] Loading task file: " << taskFilePath << std::endl;
    } 
    else 
    {
      throw std::invalid_argument(
        "[LeggedInterface] Task file not found: " + taskFilePath.string());
    }

    // Check that model file exists
    const boost::filesystem::path modelFilePath(modelFile);
    if (exists(modelFilePath)) 
    {
      std::cerr << "[LeggedInterface] Loading model file: " << modelFilePath << std::endl;
    } 
    else 
    {
      throw std::invalid_argument(
        "[LeggedInterface] Model file not found: " + modelFilePath.string());
    }

    // Check that model file exists
    const boost::filesystem::path urdfFilePath(urdfFile);
    if (exists(urdfFilePath)) 
    {
      std::cerr << "[LeggedInterface] Loading URDF file: " << urdfFilePath << std::endl;
    } 
    else 
    {
      throw std::invalid_argument(
        "[LeggedInterface] URDF file not found: " + urdfFilePath.string());
    }

    // Load model settings
    modelSettings_ = loadModelSettings(modelFile);
    const bool verbose = modelSettings_.verbose;

    // Load interface settings
    interfaceSettings_ = loadLeggedInterfaceSettings(modelFile, 
      "legged_interface_settings", verbose);

    // Load collision settings
    collisionSettings_ = loadCollisionSettings(modelFile, modelSettings_, 
      "legged_collision_settings", verbose);

    // Setup PinocchioInterface
    const auto& baseLinkName = modelSettings_.baseLinkName;
    pinocchioInterfacePtr_ = std::make_unique<PinocchioInterface>(
      createPinocchioInterfaceFromUrdfFile(urdfFile, baseLinkName));

    // Setup floating base model info
    const auto& endEffectorThreeDofNames = modelSettings_.endEffectorThreeDofNames;
    const auto& endEffectorSixDofNames = modelSettings_.endEffectorSixDofNames;
    floatingBaseModelInfo_ = createFloatingBaseModelInfo(*pinocchioInterfacePtr_, 
      endEffectorThreeDofNames, endEffectorSixDofNames);

    // Load MPC settings
    mpcSettings_ = mpc::loadSettings(taskFile, "mpc", verbose);
    ddpSettings_ = ddp::loadSettings(taskFile, "ddp", verbose);
    sqpSettings_ = sqp::loadSettings(taskFile, "sqp", verbose);
    ipmSettings_ = ipm::loadSettings(taskFile, "ipm", verbose);
    rolloutSettings_ = rollout::loadSettings(taskFile, "rollout", verbose);

    createHelperClasses();

    createReferenceManager(initTime, currentState, std::move(currentTerrainModel), 
      modelFile, urdfFile);

    createInitializer();
    
    createOptimalProblem(initTime, currentState, modelFile);

    createRollout();
  }
      
  std::shared_ptr<ReferenceManagerInterface> LeggedInterface::getReferenceManagerPtr() const
  {
    return referenceManagerPtr_;
  }

  std::vector<std::shared_ptr<SolverSynchronizedModule>> LeggedInterface::getSynchronizedModulePtrs() const
  {
    return {disturbanceModulePtr_};
  }

  LeggedReferenceManager& LeggedInterface::getLeggedReferenceManager()
  {
    return *referenceManagerPtr_;
  }

  RolloutBase& LeggedInterface::getRollout()
  {
    return *rolloutPtr_;
  }

  const OptimalControlProblem& LeggedInterface::getOptimalControlProblem() const
  {
    return optimalProblem_;
  }

  const Initializer& LeggedInterface::getInitializer() const
  {
    return *initializerPtr_;
  }

  const ddp::Settings& LeggedInterface::ddpSettings() const 
  { 
    return ddpSettings_; 
  }

  const mpc::Settings& LeggedInterface::mpcSettings() const 
  { 
    return mpcSettings_; 
  }

  const rollout::Settings& LeggedInterface::rolloutSettings() const 
  { 
    return rolloutSettings_; 
  }

  const sqp::Settings& LeggedInterface::sqpSettings() const
  { 
    return sqpSettings_; 
  }

  const ipm::Settings& LeggedInterface::ipmSettings() const
  { 
    return ipmSettings_; 
  }

  const LeggedInterface::Settings& LeggedInterface::interfaceSettings() const
  {
    return interfaceSettings_;
  }

  const ModelSettings& LeggedInterface::modelSettings() const
  {
    return modelSettings_;
  }

  const collision::CollisionSettings& LeggedInterface::collisionSettings() const
  {
    return collisionSettings_;
  }

  const FloatingBaseModelInfo& LeggedInterface::floatingBaseModelInfo() const
  {
    return floatingBaseModelInfo_;
  }

  const PinocchioInterface& LeggedInterface::pinocchioInterface() const
  {
    return *pinocchioInterfacePtr_;
  }

  const PinocchioForwardEndEffectorKinematicsCppAd& LeggedInterface::forwardKinematics() const
  {
    return *endEffectorForwardKinematics_;
  }
  
  const PinocchioForwardCollisionKinematicsCppAd& LeggedInterface::forwardCollisionKInematics() const
  {
    return *collisionForwardKinematics_;
  }
  
  const PinocchioCollisionInterface& LeggedInterface::collisionInterface() const
  {
    return *collisionInterface_;
  }
  
  const PinocchioTorqueApproximationCppAd& LeggedInterface::torqueApproximator() const
  {
    return *torqueApproximator_;
  }

  PinocchioWeightCompensator& LeggedInterface::weightCompensator()
  {
    return *weightCompensator_;
  }

  synchronization::DisturbanceSynchronizedModule& LeggedInterface::disturbanceModule()
  {
    return *dynamic_cast<synchronization::DisturbanceSynchronizedModule*>(disturbanceModulePtr_.get());
  }

  const vector_t LeggedInterface::getInitialState() const
  {
    return initialState_;
  }

  void LeggedInterface::createHelperClasses()
  {
    const bool verbose = modelSettings_.verbose;
    const bool recompileLibrariesCppAd = modelSettings_.recompileLibrariesCppAd;
    const std::string modelFolderCppAd = modelSettings_.modelFolderCppAd;

    // Setup forward kinematics
    const std::string forwardKinematicsName = "forward_kinematics";
    endEffectorForwardKinematics_ = 
      std::make_unique<PinocchioForwardEndEffectorKinematicsCppAd>(*pinocchioInterfacePtr_, 
        floatingBaseModelInfo_, forwardKinematicsName, modelFolderCppAd, 
        recompileLibrariesCppAd, verbose);

    // Setup collision interface
    collisionInterface_ = std::make_unique<collision::PinocchioCollisionInterface>(
      floatingBaseModelInfo_, modelSettings_, collisionSettings_, *pinocchioInterfacePtr_, 
      verbose);
    
    // Setup collision kinematics
    const std::string collisionKinematicsName = "collision_kinematics";
    collisionForwardKinematics_ = 
      std::make_unique<PinocchioForwardCollisionKinematicsCppAd>(*pinocchioInterfacePtr_, 
        floatingBaseModelInfo_, collisionSettings_, collisionKinematicsName, 
        modelFolderCppAd, recompileLibrariesCppAd, verbose);

    // Setup torque approximatior
    const std::string torqueApproximatorName = "torque_approximator";
    torqueApproximator_ = std::make_unique<PinocchioTorqueApproximationCppAd>(
      *pinocchioInterfacePtr_, floatingBaseModelInfo_, 
      vector_t::Zero(floatingBaseModelInfo_.actuatedDofNum), torqueApproximatorName, 
      modelFolderCppAd, recompileLibrariesCppAd, verbose);

    // Setup weight compensator

    weightCompensator_ = std::make_unique<PinocchioWeightCompensator>(
      *pinocchioInterfacePtr_, floatingBaseModelInfo_);
  }

  void LeggedInterface::createReferenceManager(scalar_t initTime, 
    const vector_t& currentState, std::unique_ptr<TerrainModel> currentTerrainModel, 
    const std::string& modelFile, const std::string& urdfFile)
  {
    const bool verbose = modelSettings_.verbose;
    const bool recompileLibrariesCppAd = modelSettings_.recompileLibrariesCppAd;
    const std::string modelFolderCppAd = modelSettings_.modelFolderCppAd;

    // Load settings for gait planner
    const auto gaitStaticSettings = loadGaitStaticParameters(modelFile, 
      modelSettings_, "gait_static_settings", verbose);

    const auto gaitDynamicSettings = loadGaitDynamicParameters(modelFile,
      modelSettings_, gaitStaticSettings, "gait_dynamic_settings", verbose);

    // Setup gait planner
    GaitPlanner gaitPlanner(gaitStaticSettings, gaitDynamicSettings, initTime);

    // Load settings for base trajectory planner
    const auto baseStaticSettings = loadBasePlannerStaticSettings(modelFile,
      "base_planner_static_settings", verbose);

    // Setup base trajectory planner
    BaseTrajectoryPlanner basePlanner(floatingBaseModelInfo_, baseStaticSettings);

    // Load inverse kinematics solver for joint trajectory planner
    const auto kinematicModelSettings = loadKinematicsModelSettings(modelFile, 
      "legged_model_settings", verbose);
    const auto inverseSolverSettings = loadInverseSolverSettings(modelFile, 
      "inverse_solver_settings", verbose);
    const auto inverseSolverName = loadInverseSolverName(modelFile, 
      "inverse_solver_name", verbose);

    MultiEndEffectorKinematics inverseKinematicsSolver(urdfFile, kinematicModelSettings, 
      inverseSolverSettings, inverseSolverName);

    // Setup joint trajectory planner
    JointTrajectoryPlanner jointPlanner(floatingBaseModelInfo_, 
      std::move(inverseKinematicsSolver));

    // Setup contact force wrench trajectory planner
    ContactForceWrenchTrajectoryPlanner contactForceWrenchPlanner(*weightCompensator_);

    // Load settings for swing trajectory planner
    const auto swingStaticSettings = loadSwingPlannerStaticSettings(modelFile, 
      "swing_planner_static_settings", verbose);
    const auto swingDynamicSettings = loadSwingPlannerDynamicSettings(modelFile, 
      floatingBaseModelInfo_, "swing_planner_dynamic_settings", verbose);
    const auto overExtensionPenaltySettings = loadOverExtensionPenaltySettings(
      modelFile, "over_extension_penalty_settings", verbose);

    // Setup over extension penalty and swing trajectory planner
    const std::string overExtensionPenaltyName = "over_extension_penalty";
    const OverExtensionPenalty overExtensionPenalty(*pinocchioInterfacePtr_,
      modelSettings_, overExtensionPenaltySettings, floatingBaseModelInfo_, 
      overExtensionPenaltyName, modelFolderCppAd, recompileLibrariesCppAd, verbose);

    SwingTrajectoryPlanner swingPlanner(floatingBaseModelInfo_, 
      swingStaticSettings, swingDynamicSettings, *endEffectorForwardKinematics_, 
      overExtensionPenalty);

    // Load reference manager settings
    const auto referenceManagerSettings = loadLeggedReferenceManagerSettings(
      modelFile, "reference_manager_settings", verbose);

    const auto gaitParameters = gaitPlanner.getDynamicParameters();

    const auto swingParameters = swingPlanner.getDynamicSettings();

    // Setup reference manager
    referenceManagerPtr_ = std::make_shared<LeggedReferenceManager>(floatingBaseModelInfo_, 
      referenceManagerSettings, std::move(gaitPlanner), std::move(swingPlanner), 
      std::move(basePlanner), std::move(jointPlanner), std::move(contactForceWrenchPlanner));

    const scalar_t timeJump = 1.0;
    
    const size_t endEffectorNum = floatingBaseModelInfo_.numThreeDofContacts 
      + floatingBaseModelInfo_.numSixDofContacts;
    const size_t standingMode = ((0x01 << (endEffectorNum)) - 1);
    const contact_flags_t standingFlags(standingMode);

    SystemObservation currentObservation;
    currentObservation.state = currentState;
      
    currentObservation.input = weightCompensator_->getInput(currentState, standingFlags);

    referenceManagerPtr_->initialize(initTime, initTime + timeJump, 
      currentObservation, standingFlags, gaitParameters, swingParameters, 
      std::move(currentTerrainModel));

  }

  void LeggedInterface::createInitializer()
  {
    // Setup initializer
    initializerPtr_ = std::make_unique<LeggedInitializer>(*weightCompensator_, 
      *referenceManagerPtr_);
  }

  void LeggedInterface::createOptimalProblem(scalar_t initTime, 
    const vector_t& currentState, const std::string& modelFile)
  {
    const bool verbose = modelSettings_.verbose;
    const bool recompileLibrariesCppAd = modelSettings_.recompileLibrariesCppAd;
    const std::string modelFolderCppAd = modelSettings_.modelFolderCppAd;

    // Get optimal, weight compensating input
    const size_t endEffectorNum = floatingBaseModelInfo_.numThreeDofContacts 
      + floatingBaseModelInfo_.numSixDofContacts;
    const size_t standingMode = ((0x01 << (endEffectorNum)) - 1);
    const contact_flags_t standingFlags(standingMode);

    const vector_t currentInput = weightCompensator_->getInput(currentState, standingFlags);
    
    // Setup cppAD dynamics
    this->creatSynchronizedModule();

    const std::string dynamicsModelName = "dynamics_model";
    optimalProblem_.dynamicsPtr = std::make_unique<LeggedDynamicsAD>(
      *pinocchioInterfacePtr_, floatingBaseModelInfo_, dynamicsModelName, 
      modelSettings_, disturbanceModule());
    
    // Setup precomputation
    optimalProblem_.preComputationPtr = std::make_unique<LeggedPrecomputation>(
      floatingBaseModelInfo_, *referenceManagerPtr_, *endEffectorForwardKinematics_, 
      *collisionForwardKinematics_, *torqueApproximator_, *weightCompensator_, 
      modelFolderCppAd, recompileLibrariesCppAd, verbose);
    
    // Setup all intermediate costs
    if(interfaceSettings_.useTrajectoryTrackingCost)
    {
      const auto baseWeights = loadBaseWeights(modelFile, "base_weights", verbose);

      const auto& model = pinocchioInterfacePtr_->getModel();
      const auto jointWeights = loadJointWeights(modelFile, floatingBaseModelInfo_, model, 
        "joint_weights", verbose);

      const auto endEffectorWeights = loadEndEffectorWeights(modelFile, modelSettings_, 
        "end_effector_weights", verbose);

      auto trajectoryTrackingCost = std::make_unique<TrajectoryTrackingCost>(
        floatingBaseModelInfo_, *referenceManagerPtr_, baseWeights, jointWeights, 
        endEffectorWeights, modelFolderCppAd, recompileLibrariesCppAd, verbose);
      
      optimalProblem_.costPtr->add("TrajectoryTrackingCost", std::move(trajectoryTrackingCost));
    }

    if(interfaceSettings_.useJointTorqueCost)
    {
      const auto& model = pinocchioInterfacePtr_->getModel();
      const auto jointWeights = loadJointTorqueWeights(modelFile, floatingBaseModelInfo_, 
        model, "joint_torque_weights", verbose);

      auto jointTorqueCost = std::make_unique<JointTorqueCost>(floatingBaseModelInfo_, 
        jointWeights);
      
      optimalProblem_.costPtr->add("JointTorqueCost", std::move(jointTorqueCost));
    }

    // Setup all constraints for each end effector
    for(size_t i = 0; i < endEffectorNum; ++i)
    {
      if(interfaceSettings_.useNormalVelocityConstraint)
      {
        auto normaVelocityConstraint = std::make_unique<NormalVelocityConstraint>(
          *referenceManagerPtr_, i);
        
        const std::string name = "NormalVelocityConstraint_" + std::to_string(i);
        optimalProblem_.equalityConstraintPtr->add(name, 
          std::move(normaVelocityConstraint));
      }
    }

    for(size_t i = 0; i < floatingBaseModelInfo_.numThreeDofContacts; ++i)
    {
      if(interfaceSettings_.useZero3DofVelocityConstraint)
      {
        auto zero3DofVelocityConstraint = std::make_unique<Zero3DofVelocityConstraint>(
          *referenceManagerPtr_, i);

        const std::string name = "Zero3DofVelocityConstraint_" + std::to_string(i);
        optimalProblem_.equalityConstraintPtr->add(name, 
          std::move(zero3DofVelocityConstraint));
      }
      
      if(interfaceSettings_.useZeroForceConstraint)
      {
        auto zeroForceConstraint = std::make_unique<ZeroForceConstraint>(
          *referenceManagerPtr_, floatingBaseModelInfo_, i);

        const std::string name = "ZeroForceConstraint_" + std::to_string(i);
        optimalProblem_.equalityConstraintPtr->add(name, 
          std::move(zeroForceConstraint));
      }
    }

    for(size_t i = floatingBaseModelInfo_.numThreeDofContacts; i < endEffectorNum; ++i)
    {
      if(interfaceSettings_.useZero6DofVelocityConstraint)
      {
        auto zero6DofVelocityConstraint = std::make_unique<Zero6DofVelocityConstraint>(
          *referenceManagerPtr_, i);

        const std::string name = "Zero6DofVelocityConstraint_" + std::to_string(i);
        optimalProblem_.equalityConstraintPtr->add(name, 
          std::move(zero6DofVelocityConstraint));
      }

      if(interfaceSettings_.useZeroWrenchConstraint)
      {
        auto zeroWrenchConstraint = std::make_unique<ZeroWrenchConstraint>(
          *referenceManagerPtr_, floatingBaseModelInfo_, i);

        const std::string name = "ZeroWrenchConstraint_" + std::to_string(i);
        optimalProblem_.equalityConstraintPtr->add(name, 
          std::move(zeroWrenchConstraint));
      }
    }

    // Setup soft constraints
    const auto frictionForceSettings = loadForceFrictionConeConfig(modelFile, 
      "force_friction_cone_soft_constraint_settings", verbose);
      
    if(interfaceSettings_.useForceFrictionConeSoftConstraint 
      && floatingBaseModelInfo_.numThreeDofContacts > 0)
    {
      auto forceFrictionConeConstraint = std::make_unique<ForceFrictionConeSoftConstraint>(
        *referenceManagerPtr_, frictionForceSettings, floatingBaseModelInfo_);
      
      const std::string name = "ForceFrictionConeSoftConstraint";
      optimalProblem_.softConstraintPtr->add(name, 
        std::move(forceFrictionConeConstraint));
    }

    const auto frictionWrenchSettings = loadWrenchFrictionConeConfig(modelFile, 
      "wrench_friction_cone_soft_constraint_settings", verbose);

    if(interfaceSettings_.useWrenchFrictionConeSoftConstraint 
      && floatingBaseModelInfo_.numSixDofContacts > 0)
    {
      auto wrenchFrictionConeSoftConstraint = std::make_unique<WrenchFrictionConeSoftConstraint>(
        *referenceManagerPtr_, frictionWrenchSettings, floatingBaseModelInfo_);
      
      const std::string name = "WrenchFrictionConeSoftConstraint";
      optimalProblem_.softConstraintPtr->add(name, 
        std::move(wrenchFrictionConeSoftConstraint));
    }

    if(interfaceSettings_.useEndEffectorPlacementSoftConstraint)
    {
      const auto endEffectorPlacementSettings = 
        loadEndEffectorPlacementSoftConstraintSettings(modelFile, modelSettings_,
          "end_effector_soft_constraint_settings", verbose);

      auto endEffectorPlacementSoftConstraint = 
        std::make_unique<EndEffectorPlacementSoftConstraint>(floatingBaseModelInfo_, 
          *referenceManagerPtr_, endEffectorPlacementSettings);

      optimalProblem_.stateSoftConstraintPtr->add("EndEffectorPlacementSoftConstraint", 
        std::move(endEffectorPlacementSoftConstraint));
    }

    if(interfaceSettings_.useJointLimitsSoftConstraint)
    {
      const auto& model = pinocchioInterfacePtr_->getModel();
      const vector_t jointMaxPositions = model.upperPositionLimit.segment(7, 
        floatingBaseModelInfo_.actuatedDofNum);
      const vector_t jointMinPositions = model.lowerPositionLimit.segment(7, 
        floatingBaseModelInfo_.actuatedDofNum);
      const vector_t jointMaxVelocity =  model.velocityLimit.segment(6, 
        floatingBaseModelInfo_.actuatedDofNum);

      const auto jointLimitsSettings = 
        loadJointLimitsSoftConstraintSettings(modelFile, 
          "joint_limits_soft_constraint_settings", verbose);

      auto jointLimitsSoftConstraint = std::make_unique<JointLimitsSoftConstraint>(
        floatingBaseModelInfo_, jointMaxPositions, jointMinPositions, jointMaxVelocity, 
        jointLimitsSettings);

      optimalProblem_.softConstraintPtr->add("JointLimitsSoftConstraint", 
        std::move(jointLimitsSoftConstraint));
    }

    if(interfaceSettings_.useJointTorqueLimitsSoftConstraint)
    {
      const auto& model = pinocchioInterfacePtr_->getModel();
      const vector_t jointMaxTorque =  model.effortLimit.segment(6, 
        floatingBaseModelInfo_.actuatedDofNum);
      
      // Get nominal joint torques
      const vector_t nominalTorques = torqueApproximator_->getValue(currentState, 
        currentInput);

      const auto jointTorqueLimitsSettings = 
        loadJointTorqueLimitsSoftConstraintSettings(modelFile, 
          "joint_torque_limits_soft_constraint_settings", verbose);

      auto jointTorqueLimitsSoftConstraint = std::make_unique<JointTorqueLimitsSoftConstraint>(
        floatingBaseModelInfo_, nominalTorques, jointMaxTorque, jointTorqueLimitsSettings);

      optimalProblem_.softConstraintPtr->add("JointTorqueLimitsSoftConstraint", 
        std::move(jointTorqueLimitsSoftConstraint));
    }

    if(interfaceSettings_.useTerrainAvoidanceSoftConstraint)
    {
      const auto terrainAvoidanceSettings = 
        loadTerrainAvoidanceSoftConstraintSettings(modelFile, 
          "terrain_avoidance_soft_constraint_settings", verbose);

      auto terrainAvoidanceSoftConstraint = std::make_unique<TerrainAvoidanceSoftConstraint>(
        floatingBaseModelInfo_, collisionSettings_, *collisionInterface_, 
        *referenceManagerPtr_, terrainAvoidanceSettings);

      optimalProblem_.stateSoftConstraintPtr->add("TerrainAvoidanceSoftConstraint", 
        std::move(terrainAvoidanceSoftConstraint));
    }

    if(interfaceSettings_.useSelfCollisionAvoidanceSoftConstraint)
    {
      const auto selfCollisionAvoidanceSettings = 
        loadSelfCollisionAvoidanceSoftConstraintSettings(modelFile, 
          "self_collision_avoidance_soft_constraint_settings", verbose);

      auto selfCollisionAvoidanceSoftConstraint = 
        std::make_unique<SelfCollisionAvoidanceSoftConstraint>(
          floatingBaseModelInfo_, collisionSettings_, *collisionInterface_, 
          *referenceManagerPtr_, selfCollisionAvoidanceSettings);

      optimalProblem_.stateSoftConstraintPtr->add("SelfCollisionAvoidanceSoftConstraint", 
        std::move(selfCollisionAvoidanceSoftConstraint));
    }

    if(interfaceSettings_.useNormalOrientationSoftConstraint 
      && floatingBaseModelInfo_.numSixDofContacts > 0)
    {
      const auto normalOrientationSettings = loadNormalOrientationSoftConstraintSettings(
        modelFile, "normal_orientation_soft_constraint_settings", verbose);

      auto normalOrientationSoftConstraint = 
        std::make_unique<NormalOrientationSoftConstraint>(*referenceManagerPtr_, 
          normalOrientationSettings, floatingBaseModelInfo_, 
          modelFolderCppAd, recompileLibrariesCppAd, verbose);

      optimalProblem_.stateSoftConstraintPtr->add("NormalOrientationSoftConstraint", 
        std::move(normalOrientationSoftConstraint));
    }

    // Setup all terminal costs
    if(interfaceSettings_.useTerminalTrackingCost)
    {
      optimalProblem_.targetTrajectoriesPtr = &referenceManagerPtr_->getTargetTrajectories();

      auto lqrSolution = continuous_time_lqr::solve(optimalProblem_, initTime, 
        currentState, currentInput);
      
      const auto weightMatrix = lqrSolution.valueFunction;

      auto terminalTrackingCost = std::make_unique<TerminalTrackingCost>(weightMatrix, 
        floatingBaseModelInfo_);

      optimalProblem_.finalCostPtr->add("TerminalTrackingCost", 
        std::move(terminalTrackingCost));
    }
  }

  void LeggedInterface::creatSynchronizedModule()
  {
    disturbanceModulePtr_ = std::make_shared<synchronization::DisturbanceSynchronizedModule>();
  }

  void LeggedInterface::createRollout()
  {
    rolloutPtr_ = std::make_unique<TimeTriggeredRollout>(*optimalProblem_.dynamicsPtr, 
      rolloutSettings_);
  }

   LeggedInterface::Settings loadLeggedInterfaceSettings(const std::string& filename,
    const std::string& fieldName, bool verbose)
  {
    LeggedInterface::Settings settings;

    boost::property_tree::ptree pt;
    read_info(filename, pt);

    if(verbose) 
    {
      std::cerr << "\n #### Legged Locomotion MPC Common Model Settings:";
      std::cerr << "\n #### =============================================================================\n";
    }

    loadData::loadPtreeValue(pt, settings.useTrajectoryTrackingCost, 
      fieldName + ".useTrajectoryTrackingCost", verbose);
    loadData::loadPtreeValue(pt, settings.useTerminalTrackingCost, 
      fieldName + ".useTerminalTrackingCost", verbose);
    loadData::loadPtreeValue(pt, settings.useJointTorqueCost, 
      fieldName + ".useJointTorqueCost", verbose);
    loadData::loadPtreeValue(pt, settings.useNormalVelocityConstraint, 
      fieldName + ".useNormalVelocityConstraint", verbose);
    loadData::loadPtreeValue(pt, settings.useForceFrictionConeSoftConstraint, 
      fieldName + ".useForceFrictionConeSoftConstraint", verbose);
    loadData::loadPtreeValue(pt, settings.useZero3DofVelocityConstraint, 
      fieldName + ".useZero3DofVelocityConstraint", verbose);
    loadData::loadPtreeValue(pt, settings.useZeroForceConstraint, 
      fieldName + ".useZeroForceConstraint", verbose);
    loadData::loadPtreeValue(pt, settings.useWrenchFrictionConeSoftConstraint, 
      fieldName + ".useWrenchFrictionConeSoftConstraint", verbose);
    loadData::loadPtreeValue(pt, settings.useZero6DofVelocityConstraint, 
      fieldName + ".useZero6DofVelocityConstraint", verbose);
    loadData::loadPtreeValue(pt, settings.useZeroWrenchConstraint, 
      fieldName + ".useZeroWrenchConstraint", verbose);
    loadData::loadPtreeValue(pt, settings.useEndEffectorPlacementSoftConstraint, 
      fieldName + ".useEndEffectorPlacementSoftConstraint", verbose);
    loadData::loadPtreeValue(pt, settings.useJointLimitsSoftConstraint, 
      fieldName + ".useJointLimitsSoftConstraint", verbose);
    loadData::loadPtreeValue(pt, settings.useJointTorqueLimitsSoftConstraint, 
      fieldName + ".useJointTorqueLimitsSoftConstraint", verbose);
    loadData::loadPtreeValue(pt, settings.useTerrainAvoidanceSoftConstraint, 
      fieldName + ".useTerrainAvoidanceSoftConstraint", verbose);
    loadData::loadPtreeValue(pt, settings.useSelfCollisionAvoidanceSoftConstraint, 
      fieldName + ".useSelfCollisionAvoidanceSoftConstraint", verbose);
    loadData::loadPtreeValue(pt, settings.useNormalOrientationSoftConstraint, 
      fieldName + ".useNormalOrientationSoftConstraint", verbose);

    if(verbose) 
    {
      std::cerr << " #### =============================================================================" <<
      std::endl;
    }

    return settings;
  }
} // namespace legged_locomotion_mpc