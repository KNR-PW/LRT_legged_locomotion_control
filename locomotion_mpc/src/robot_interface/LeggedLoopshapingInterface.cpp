#include <legged_locomotion_mpc/robot_interface/LeggedLoopshapingInterface.hpp>

#include <ocs2_core/loopshaping/LoopshapingPropertyTree.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>
#include <ocs2_oc/approximate_model/LinearQuadraticApproximator.h>

namespace legged_locomotion_mpc
{ 

  using namespace ocs2;
  using namespace terrain_model;
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  LeggedLoopshapingInterface::LeggedLoopshapingInterface(
    std::unique_ptr<LeggedInterface> leggedInterfacePtr,
    std::shared_ptr<LoopshapingDefinition> loopshapingDefinitionPtr):
      LoopshapingRobotInterface(std::move(leggedInterfacePtr), 
        std::move(loopshapingDefinitionPtr)) 
  {
    LeggedInterface& leggedInterface = getLeggedInterface();

    const auto& modelInfo = leggedInterface.floatingBaseModelInfo();

    const size_t endEffectorNum = modelInfo.numThreeDofContacts 
      + modelInfo.numSixDofContacts;
    const size_t standingMode = ((0x01 << (endEffectorNum)) - 1);
    const contact_flags_t standingFlags(standingMode);

    auto& weightCompensator = leggedInterface.weightCompensator();

    const auto& initalSystemState = leggedInterface.getInitialState();

    const vector_t initialInput = weightCompensator.getInput(initalSystemState, 
      standingFlags);

    vector_t initialFilterState;
    vector_t initialFilterInput;
    getLoopshapingDefinition()->getFilterEquilibrium(initialInput, initialFilterState, initialFilterInput);
    initialState_ = getLoopshapingDefinition()->concatenateSystemAndFilterState(
      initalSystemState, initialFilterState);

    loopshapingRolloutPtr_.reset(
      new TimeTriggeredRollout(*getOptimalControlProblem().dynamicsPtr, leggedInterface.rolloutSettings()));
  }
  
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  LeggedInterface& LeggedLoopshapingInterface::getLeggedInterface()
  {
    return const_cast<LeggedInterface&>(get<LeggedInterface>());
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  const vector_t& LeggedLoopshapingInterface::getInitialState() const
  {
    return initialState_;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  RolloutBase& LeggedLoopshapingInterface::getRollout()
  {
    return *loopshapingRolloutPtr_;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  LeggedLoopshapingInterface makeLeggedLoopshapingInterface(
    scalar_t initTime, const vector_t& currentSystemState, 
    std::unique_ptr<TerrainModel> currentTerrainModel, const std::string& taskFile, 
    const std::string& modelFile, const std::string& urdfFile, 
    const std::string& loopshapingDefinitionFile)
  {
    auto loopShapingDefinition = loopshaping_property_tree::load(loopshapingDefinitionFile);
    
    std::unique_ptr<LeggedInterface> leggedInterfacePtr = std::make_unique<LeggedInterface>(
      initTime, currentSystemState, std::move(currentTerrainModel), taskFile, modelFile, urdfFile);
    

    // Get cost matrix for loopshaping using starting state and input
    const auto& optimalProblem = leggedInterfacePtr->getOptimalControlProblem();

    const auto& modelInfo = leggedInterfacePtr->floatingBaseModelInfo();

    const size_t endEffectorNum = modelInfo.numThreeDofContacts 
      + modelInfo.numSixDofContacts;
    const size_t standingMode = ((0x01 << (endEffectorNum)) - 1);
    const contact_flags_t standingFlags(standingMode);

    auto& weightCompensator = leggedInterfacePtr->weightCompensator();

    const auto& initalSystemState = leggedInterfacePtr->getInitialState();

    const vector_t initialInput = weightCompensator.getInput(initalSystemState, 
      standingFlags);

    const auto nominalCostApproximation = approximateCost(optimalProblem, 0.0, 
      initalSystemState, initialInput);

    loopShapingDefinition->costMatrix() = nominalCostApproximation.dfduu;

    return LeggedLoopshapingInterface(std::move(leggedInterfacePtr), 
      std::move(loopShapingDefinition));
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  std::unique_ptr<LeggedLoopshapingInterface> makeLeggedLoopshapingInterfacePointer(
    ocs2::scalar_t initTime, const ocs2::vector_t& currentSystemState, 
    std::unique_ptr<terrain_model::TerrainModel> currentTerrainModel, 
    const std::string& taskFile, const std::string& modelFile, const std::string& urdfFile, 
    const std::string& loopshapingDefinitionFile)
  {
    auto loopShapingDefinition = loopshaping_property_tree::load(loopshapingDefinitionFile);
    
    std::unique_ptr<LeggedInterface> leggedInterfacePtr = std::make_unique<LeggedInterface>(
      initTime, currentSystemState, std::move(currentTerrainModel), taskFile, modelFile, urdfFile);
    

    // Get cost matrix for loopshaping using starting state and input
    const auto& optimalProblem = leggedInterfacePtr->getOptimalControlProblem();

    const auto& modelInfo = leggedInterfacePtr->floatingBaseModelInfo();

    const size_t endEffectorNum = modelInfo.numThreeDofContacts 
      + modelInfo.numSixDofContacts;
    const size_t standingMode = ((0x01 << (endEffectorNum)) - 1);
    const contact_flags_t standingFlags(standingMode);

    auto& weightCompensator = leggedInterfacePtr->weightCompensator();

    const auto& initalSystemState = leggedInterfacePtr->getInitialState();

    const vector_t initialInput = weightCompensator.getInput(initalSystemState, 
      standingFlags);

    const auto nominalCostApproximation = approximateCost(optimalProblem, 0.0, 
      initalSystemState, initialInput);

    loopShapingDefinition->costMatrix() = nominalCostApproximation.dfduu;

    return std::make_unique<LeggedLoopshapingInterface>(std::move(leggedInterfacePtr), 
      std::move(loopShapingDefinition));
  }
} // namespace legged_locomotion_mpc