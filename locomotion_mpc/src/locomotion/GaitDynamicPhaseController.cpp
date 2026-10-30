#include <legged_locomotion_mpc/locomotion/GaitDynamicPhaseController.hpp>

#include <algorithm>
#include <iostream>

#include <ocs2_core/misc/Numerics.h>

#include <legged_locomotion_mpc/common/Utils.hpp>
#include <legged_locomotion_mpc/locomotion/GaitCommon.hpp>

namespace legged_locomotion_mpc
{
  namespace locomotion
  {
    using namespace ocs2;

    GaitDynamicPhaseController::GaitDynamicPhaseController(scalar_t initTime,
      const GaitStaticParameters& staticParams,
      const GaitDynamicParameters& initDynamicParams)
        : staticParams_(staticParams)
    {
      // Allocate memory in advance 
      eventTimes_.reserve(Definitions::DEFAULT_BUFFER_SIZE);
      dynamicParamsVec_.reserve(Definitions::DEFAULT_BUFFER_SIZE);
      cachedPhase_.reserve(Definitions::DEFAULT_BUFFER_SIZE);
      
      eventTimes_.push_back(initTime);
      dynamicParamsVec_.push_back(initDynamicParams);
      cachedPhase_.push_back(staticParams.initPhase);
    }

    std::vector<scalar_t> GaitDynamicPhaseController::getPhasesAtTime(scalar_t time) const
    {
      assert(time >= eventTimes_.front());

      std::vector<scalar_t> returnPhases(staticParams_.endEffectorNumber);

      if(numerics::almost_eq(time, eventTimes_.front(), SCALAR_EPSILON))
      {
        const scalar_t returnPhase = cachedPhase_.front();
        const auto& offsets = dynamicParamsVec_.front().phaseOffsets;

        returnPhases[0] = normalizePhase(returnPhase);

        for(int i = 1; i < staticParams_.endEffectorNumber; ++i)
        {
          returnPhases[i] = normalizePhase(returnPhase + offsets[i - 1]);
        }

        return returnPhases;
      }

      // Find index of time larger that query time
      // - SCALAR_EPSILON -> Compatibility with ModeSchedule
      const size_t index = utils::findIndexInTimeArray(eventTimes_, time - SCALAR_EPSILON);
      
      // Add phase between index - 1 time and query time
      const auto frequency = dynamicParamsVec_[index - 1].steppingFrequency;
      scalar_t returnPhase = cachedPhase_[index - 1] + (time - eventTimes_[index - 1]) * frequency;

      returnPhases[0] = normalizePhase(returnPhase);
      const auto& offsets = dynamicParamsVec_[index - 1].phaseOffsets;
      for(int i = 1; i < staticParams_.endEffectorNumber; ++i)
      {
        returnPhases[i] = normalizePhase(returnPhase + offsets[i - 1]);
      }

      return returnPhases;
    }

    contact_flags_t GaitDynamicPhaseController::getContactFlagsAtTime(scalar_t time) const
    {

      // - SCALAR_EPSILON -> Compatibility with ModeSchedule
      assert(time >= eventTimes_.front());

      contact_flags_t returnFlags(staticParams_.endEffectorNumber);

      if(numerics::almost_eq(time, eventTimes_.front(), SCALAR_EPSILON))
      {
        const scalar_t returnPhase = cachedPhase_.front();
        const scalar_t swingRatio = dynamicParamsVec_.front().swingRatio;
        const auto& offsets = dynamicParamsVec_.front().phaseOffsets;

        returnFlags[0] = normalizePhase(returnPhase - SCALAR_EPSILON) >= swingRatio;
        
        for(int i = 1; i < staticParams_.endEffectorNumber; ++i)
        {
          returnFlags[i] = normalizePhase(returnPhase + offsets[i - 1] - SCALAR_EPSILON) >= swingRatio;
        }

        return returnFlags;
      }
      
      // Find index of time larger that query time
      const size_t index = utils::findIndexInTimeArray(eventTimes_, time - SCALAR_EPSILON);
      
      // Add phase between index - 1 time and query time
      const auto frequency = dynamicParamsVec_[index - 1].steppingFrequency;
      scalar_t returnPhase = cachedPhase_[index - 1] + (time - eventTimes_[index - 1]) * frequency;

      const auto& offsets = dynamicParamsVec_[index - 1].phaseOffsets;
      const scalar_t swingRatio = dynamicParamsVec_[index - 1].swingRatio;

      returnFlags[0] = normalizePhase(returnPhase - SCALAR_EPSILON) >= swingRatio;
      for(int i = 1; i < staticParams_.endEffectorNumber; ++i)
      {
        returnFlags[i] = normalizePhase(returnPhase + offsets[i - 1] - SCALAR_EPSILON) >= swingRatio;
      }
      return returnFlags;
    }

    const GaitDynamicParameters& GaitDynamicPhaseController::getDynamicParametersAtTime(
      scalar_t time) const
    {
      assert(time >= eventTimes_.front());

      if(numerics::almost_eq(time, eventTimes_.front(), SCALAR_EPSILON))
      {
        return dynamicParamsVec_.front();
      }

      // Find index of time larger that query time
      const size_t index = utils::findIndexInTimeArray(eventTimes_, time);

      return dynamicParamsVec_[index - 1];
    }

    void GaitDynamicPhaseController::remove(scalar_t time)
    {
      // Find index of time larger that query time
     const size_t index = utils::findIndexInTimeArray(eventTimes_, time);

      if(index == 0) return;

      // delete the old time and frequency values to time smaller than query time
      eventTimes_.erase(eventTimes_.begin(), eventTimes_.begin() + index - 1);
      dynamicParamsVec_.erase(dynamicParamsVec_.begin(), dynamicParamsVec_.begin() + index - 1);
      cachedPhase_.erase(cachedPhase_.begin(), cachedPhase_.begin() + index - 1);
    }

    void GaitDynamicPhaseController::update(scalar_t newTime, 
      const GaitDynamicParameters& newDynamicParams)
    {
      assert(newTime > eventTimes_.back());

      // Calculate new cached phase
      scalar_t newCachedPhase = normalizePhase(cachedPhase_.back() + (newTime - eventTimes_.back()) * dynamicParamsVec_.back().steppingFrequency);
      eventTimes_.push_back(newTime);
      dynamicParamsVec_.push_back(newDynamicParams);
      cachedPhase_.push_back(newCachedPhase);
    }

    void GaitDynamicPhaseController::update(scalar_t newTime,
      GaitDynamicParameters&& newDynamicParams)
    {
      assert(newTime > eventTimes_.back());

      // Calculate new cached phase
      scalar_t newCachedPhase = normalizePhase(cachedPhase_.back() + (newTime - eventTimes_.back()) * dynamicParamsVec_.back().steppingFrequency);
      eventTimes_.push_back(newTime);
      dynamicParamsVec_.push_back(std::move(newDynamicParams));
      cachedPhase_.push_back(newCachedPhase);
    }
  }
}