#include <legged_locomotion_mpc/locomotion/GaitPlanner.hpp>

#include <algorithm>
#include <limits>
#include <iostream>

#include <ocs2_core/misc/Numerics.h>

#include <legged_locomotion_mpc/common/Utils.hpp>
#include <legged_locomotion_mpc/locomotion/ModeSequenceTemplate.hpp>
#include <legged_locomotion_mpc/locomotion/ModeDynamicSequenceTemplate.hpp>

namespace legged_locomotion_mpc
{
  namespace locomotion
  {
    using namespace ocs2;

    GaitPlanner::GaitPlanner(const GaitStaticParameters& staticParams,
      const GaitDynamicParameters& initDynamicParams, scalar_t initTime):
        currentStartTime_(initTime),
        currentDynamicParamsTime_(initTime),
        staticParams_(staticParams),
        dynamicParams_(initDynamicParams),
        phaseController_(initTime, staticParams, initDynamicParams)
    {
      if(initTime < 0.0)
      {
        std::string message = "[GaitPlanner]: Time lower than 0!";
        throw std::invalid_argument(message);
      }

      if(staticParams_.initPhase < 0.0 || staticParams_.initPhase > 1.0)
      {
        std::string message = "[GaitPlanner]: Phase lower than 0.0 or higher than 1.0!";
        throw std::invalid_argument(message);
      }

      // Create first simple modeSchedule
      modeSchedule_.clear();
      modeSchedule_.eventTimes = {initTime};

      const contact_flags_t currentFlags = getContactFlagsAtTime(initTime);
      const size_t currentMode = contactFlags2ModeNumber(currentFlags);
      modeSchedule_.modeSequence = {currentMode, currentMode};
    }

    ModeSchedule GaitPlanner::getModeSchedule(scalar_t startTime,
      scalar_t finalTime) 
    {
      assert(startTime >= currentStartTime_);
      assert(finalTime > startTime);
      assert(!numerics::almost_eq(startTime, finalTime, SCALAR_EPSILON));

      currentStartTime_ = startTime;

      phaseController_.remove(startTime);

      auto& eventTimes = modeSchedule_.eventTimes;
      auto& modeSequence = modeSchedule_.modeSequence;

      const size_t startIndex = utils::findIndexInTimeArray(eventTimes, startTime);

      if(startIndex > 0)
      {
        // delete the old logic from index behdind startTime
        eventTimes.erase(eventTimes.begin(), eventTimes.begin() + startIndex);
        modeSequence.erase(modeSequence.begin(), modeSequence.begin() + startIndex);
      }

      if(!modeSequence.empty())
      {
        const size_t endIndex = utils::findIndexInTimeArray(eventTimes, finalTime);

        if(endIndex < eventTimes.size())
        {
          // delete the old logic from index after finalTime
          eventTimes.erase(eventTimes.begin() + endIndex, eventTimes.end());
          modeSequence.erase(modeSequence.begin() + endIndex, modeSequence.end());
        }
        else
        {
          // delete the stance phase
          modeSequence.erase(modeSequence.end() - 1, modeSequence.end());
        }
      }

      const scalar_t tilingTime = eventTimes.empty() ? startTime : eventTimes.back();

      const scalar_t startPhase = phaseController_.getPhasesAtTime(tilingTime)[0];

      ModeSequenceTemplate modeSequenceTemplate = getDynamicModeSequenceTemplate(
        startPhase, finalTime - tilingTime, staticParams_, dynamicParams_);

      const auto& templateTimes = modeSequenceTemplate.switchingTimes;
      const auto& templateModeSequence = modeSequenceTemplate.modeSequence;
      const size_t numTemplateSubsystems = modeSequenceTemplate.modeSequence.size();

      assert(numTemplateSubsystems != 0);

      const contact_flags_t currentFlags = getContactFlagsAtTime(startTime);
      if(eventTimes.empty())
      {
        eventTimes.push_back(startTime);
        modeSequence.push_back(contactFlags2ModeNumber(currentFlags));
      }

      // concatenate from index
      while (eventTimes.back() < finalTime) 
      {
        for (size_t i = 0; i < templateModeSequence.size(); i++) 
        {
          scalar_t deltaTime = templateTimes[i + 1] - templateTimes[i];
          if(templateModeSequence[i] == modeSequence.back())
          {
            eventTimes.back() += deltaTime;
          }
          else
          {
            modeSequence.push_back(templateModeSequence[i]);
            eventTimes.push_back(eventTimes.back() + deltaTime);
          }
        } // end of i loop
      } // end of while loop

      const size_t standingMode = ((0x01 << (staticParams_.endEffectorNumber)) - 1);
      modeSequence.push_back(standingMode);

      return modeSchedule_;
    }

    GaitFlags GaitPlanner::updateCurrentContacts(scalar_t time, 
      const contact_flags_t& currentContacts)
    {
      const contact_flags_t plannedFlags = getContactFlagsAtTime(time);

      if(plannedFlags == currentContacts)
      {
        return GaitFlags::OK; // As planned, early return
      } 

      if(contactFlags2ModeNumber(currentContacts) == 0)
      {
        return GaitFlags::IN_THE_AIR; // Cricital error, robot is in the air!
      }

      return GaitFlags::ERROR;
    }

    void GaitPlanner::updateDynamicParameters(scalar_t time,
      const GaitDynamicParameters& dynamicParams)
    {
      assert(time >= currentDynamicParamsTime_);

      const auto clampedParams = clampGaitDynamicParams(dynamicParams);

      // Compare clamped parameters, stored parameters are also clamped
      if(clampedParams == dynamicParams_) return;

      // Check mode with new parameters at time t
      const scalar_t basePhase = phaseController_.getPhasesAtTime(time)[0];
      const scalar_t swingRatio = clampedParams.swingRatio;
      const auto& offsets = clampedParams.phaseOffsets;
      std::vector<scalar_t> newPhase(staticParams_.endEffectorNumber);
      contact_flags_t newFlags;

      newPhase[0] = normalizePhase(basePhase - SCALAR_EPSILON);
      newFlags[0] = newPhase[0] >= swingRatio;
      for(size_t i = 1; i < staticParams_.endEffectorNumber; ++i)
      {
        newPhase[i] = normalizePhase(basePhase + offsets[i - 1] - SCALAR_EPSILON);
        newFlags[i] = newPhase[i] >= swingRatio;
      }

      const size_t newMode = contactFlags2ModeNumber(newFlags);

      auto& eventTimes = modeSchedule_.eventTimes;
      auto& modeSequence = modeSchedule_.modeSequence;

      // Get old mode at this time
      size_t endIndex = 0;
      size_t currentMode = 0;

      if(!modeSequence.empty())
      {
        endIndex = utils::findIndexInTimeArray(eventTimes, time);
        currentMode = modeSequence[endIndex];

        if(endIndex < eventTimes.size())
        {
          // delete the old logic from index after finalTime
          eventTimes.erase(eventTimes.begin() + endIndex, eventTimes.end());
          modeSequence.erase(modeSequence.begin() + endIndex + 1, modeSequence.end());
        }
      }

      // Smooth transition, seems good
      if(newMode == currentMode)
      {
        currentDynamicParamsTime_ = time;
        phaseController_.update(currentDynamicParamsTime_, clampedParams);

        const scalar_t gaitPeriod = 1 / clampedParams.steppingFrequency;

        scalar_t minTimeToNextMode = std::numeric_limits<scalar_t>::max();
        for(size_t i = 0; i < staticParams_.endEffectorNumber; ++i)
        {
          const scalar_t timeToNextMode = getTimeToNextMode(newPhase[i], swingRatio, gaitPeriod);
          if(timeToNextMode < minTimeToNextMode)
          {
            minTimeToNextMode = timeToNextMode;
          }
        }

        eventTimes.push_back(currentDynamicParamsTime_ + minTimeToNextMode);
        const size_t standingMode = ((0x01 << (staticParams_.endEffectorNumber)) - 1);
        modeSequence.push_back(standingMode);

        dynamicParams_ = clampedParams;
      }
      // Not the same mode
      else
      {
        // Last event before time (later events were erased above)
        if(!eventTimes.empty() &&
          std::abs(time - eventTimes.back()) < Definitions::MIN_TIME_BETWEEN_CHANGES)
        {
          // Move time of dynamic params change by MIN_TIME_BETWEEN_CHANGES
          currentDynamicParamsTime_ = time + Definitions::MIN_TIME_BETWEEN_CHANGES;
        }
        else
        {
          currentDynamicParamsTime_ = time;
        }

        phaseController_.update(currentDynamicParamsTime_, clampedParams);

        eventTimes.push_back(currentDynamicParamsTime_);
        modeSequence.push_back(newMode);

        dynamicParams_ = clampedParams;
      }
    }

    const GaitStaticParameters& GaitPlanner::getStaticParameters()
    {
      return staticParams_;
    }

    const GaitDynamicParameters& GaitPlanner::getDynamicParameters()
    {
      return dynamicParams_;
    }

    std::vector<scalar_t> GaitPlanner::getPhasesAtTime(scalar_t time) const
    {
      return phaseController_.getPhasesAtTime(time);
    }

    std::vector<std::vector<scalar_t>> GaitPlanner::getPhasesAtTimes(
      const std::vector<scalar_t>& times) const
    {
      const size_t referenceSize = times.size();
      std::vector<std::vector<scalar_t>> phasesTrajectory;
      phasesTrajectory.reserve(referenceSize);
      for(const auto time: times)
      {
        std::vector<scalar_t> currentPhases = getPhasesAtTime(time);
        phasesTrajectory.push_back(std::move(currentPhases));
      }
      return phasesTrajectory;
    }

    contact_flags_t GaitPlanner::getContactFlagsAtTime(scalar_t time) const
    {
      return phaseController_.getContactFlagsAtTime(time);
    }

    std::vector<contact_flags_t> GaitPlanner::getContactFlagsAtTimes(
      const std::vector<scalar_t>& times) const
    {
      const size_t referenceSize = times.size();
      std::vector<contact_flags_t> contactFlagsTrajectory;
      contactFlagsTrajectory.reserve(referenceSize);
      for(const auto time: times)
      {
        contact_flags_t currentFlags = getContactFlagsAtTime(time);
        contactFlagsTrajectory.push_back(std::move(currentFlags));
      }
      return contactFlagsTrajectory;
    }

    GaitDynamicParameters GaitPlanner::clampGaitDynamicParams(
      const GaitDynamicParameters& dynamicParams)
    {
      auto newDynamicsParams = dynamicParams;

      // Stepping frequency cannot be higher than maximum
      newDynamicsParams.steppingFrequency = std::min(newDynamicsParams.steppingFrequency,
        staticParams_.maximumSteppingFrequency);

      auto& phaseOffsets = newDynamicsParams.phaseOffsets;
      const scalar_t steppingFrequency = newDynamicsParams.steppingFrequency;
      const scalar_t gaitPeriod = 1.0 / steppingFrequency;

      const size_t offsetSize = staticParams_.endEffectorNumber - 1;
      std::vector<scalar_t> timeOffsets;
      timeOffsets.reserve(offsetSize);

      // Check if other phases are close to each other
      for(size_t i = 0; i < offsetSize; ++i)
      {
        timeOffsets.emplace_back(gaitPeriod * phaseOffsets[i]);
      }

      for(size_t i = 0; (i + 1) < offsetSize; ++i)
      {
        for(size_t j = (i + 1); j < offsetSize; ++j)
        {
          if(std::abs(timeOffsets[i] - timeOffsets[j]) <
            Definitions::MIN_TIME_BETWEEN_CHANGES)
          {
            phaseOffsets[j] = phaseOffsets[i];
            timeOffsets[j] = timeOffsets[i];
          }
        }
      }

      return newDynamicsParams;
    }
  }
}