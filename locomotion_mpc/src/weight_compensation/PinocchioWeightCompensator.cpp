#include <legged_locomotion_mpc/weight_compensation/PinocchioWeightCompensator.hpp>

#include <functional>

#include <floating_base_model/AccessHelperFunctions.hpp>
#include <floating_base_model/ModelHelperFunctions.hpp>
#include <floating_base_model/FloatingBaseModelPinocchioMapping.hpp>

#include <legged_locomotion_mpc/common/Utils.hpp>
#include <legged_locomotion_mpc/common/ModelHelperFunctions.hpp>

namespace legged_locomotion_mpc 
{

  using namespace ocs2;
  using namespace floating_base_model;
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  PinocchioWeightCompensator::PinocchioWeightCompensator(
    const PinocchioInterface& pinocchioInterface,
    const FloatingBaseModelInfo& info): 
      info_(info), pinocchioInterface_(pinocchioInterface), mapping_(info) 
  {
    mapping_.setPinocchioInterface(pinocchioInterface_);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  PinocchioWeightCompensator::PinocchioWeightCompensator(
    const PinocchioWeightCompensator& rhs)
      : info_(rhs.info_), pinocchioInterface_(rhs.pinocchioInterface_), mapping_(rhs.info_) 
  {
    mapping_.setPinocchioInterface(pinocchioInterface_);
  }

  PinocchioWeightCompensator::PinocchioWeightCompensator(PinocchioWeightCompensator&& rhs):
    info_(rhs.info_), 
    pinocchioInterface_(std::move(rhs.pinocchioInterface_)), mapping_(rhs.info_)
  {
    mapping_.setPinocchioInterface(pinocchioInterface_);
  }
  
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  PinocchioWeightCompensator* PinocchioWeightCompensator::clone() const 
  {
    return new PinocchioWeightCompensator(*this);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  std::vector<vector6_t> PinocchioWeightCompensator::getContactWrenches(
    const vector_t& state, const contact_flags_t& contactFlags)
  {
    assert(state.size() == info_.stateDim);

    // Special case, only 3 DoF contacts
    if(info_.numSixDofContacts == 0)
    {
      const vector_t input = utils::weightCompensatingInput(info_, contactFlags);
      std::vector<vector6_t> wrenches(info_.numThreeDofContacts, vector6_t::Zero());
      for(size_t i = 0; i < info_.numThreeDofContacts; ++i)
      {
        wrenches[i].block<3, 1>(0, 0) = access_helper_functions::getContactForces(input, i, info_);
      }
      return wrenches;
    }

    const vector_t q = mapping_.getPinocchioJointPosition(state);
    
    return model_helper_functions::computeWeightCompensationWrenches(
      pinocchioInterface_, info_, q, contactFlags);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  vector_t PinocchioWeightCompensator::getInput(
    const vector_t& state, const contact_flags_t& contactFlags)
  {
    assert(state.size() == info_.stateDim);

    // Special case, only 3 DoF contacts
    if(info_.numSixDofContacts == 0)
    {
      return utils::weightCompensatingInput(info_, contactFlags);
    }
    
    vector_t input = vector_t::Zero(info_.inputDim);
    const vector_t q = mapping_.getPinocchioJointPosition(state);
    const auto wrenches = model_helper_functions::computeWeightCompensationWrenches(
      pinocchioInterface_, info_, q, contactFlags);

    for(size_t i = 0; i < info_.numThreeDofContacts; ++i)
    {
      access_helper_functions::getContactForces(input, i, info_) = wrenches[i].block<3, 1>(0, 0);
    }

    for(size_t i = info_.numThreeDofContacts; i < info_.numThreeDofContacts + info_.numSixDofContacts; ++i)
    {
      access_helper_functions::getContactWrenches(input, i, info_) = wrenches[i];
    }

    return input;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void PinocchioWeightCompensator::appendInput(
    const vector_t& state, vector_t& input, const contact_flags_t& contactFlags)
  {
    assert(state.size() == info_.stateDim);
    assert(input.size() == info_.inputDim);

    // Special case, only 3 DoF contacts
    if(info_.numSixDofContacts == 0)
    {
      return utils::weightCompensatingAppendInput(input, info_, contactFlags);
    }

    const vector_t q = mapping_.getPinocchioJointPosition(state);
    const auto wrenches = model_helper_functions::computeWeightCompensationWrenches(
      pinocchioInterface_, info_, q, contactFlags);

    for(size_t i = 0; i < info_.numThreeDofContacts; ++i)
    {
      access_helper_functions::getContactForces(input, i, info_) = wrenches[i].block<3, 1>(0, 0);
    }

    for(size_t i = info_.numThreeDofContacts; i < info_.numThreeDofContacts + info_.numSixDofContacts; ++i)
    {
      access_helper_functions::getContactWrenches(input, i, info_) = wrenches[i];
    }
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  const FloatingBaseModelInfo& PinocchioWeightCompensator::getInfo() const
  {
    return info_;
  }
} // namespace legged_locomotion_mpc