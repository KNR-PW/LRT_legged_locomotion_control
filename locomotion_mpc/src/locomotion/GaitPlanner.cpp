#include <legged_locomotion_mpc/locomotion/GaitPlanner.hpp>

#include <algorithm>
#include <cmath>
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
        targetDynamicParams_(initDynamicParams),
        lastStepTime_(initTime),
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

      // Phase after tiling time, it differs at the start of the phase after re-phasing
      const scalar_t startPhase = phaseController_.getBasePhaseAfterTime(tilingTime);

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
      targetDynamicParams_ = clampGaitDynamicParams(dynamicParams);
      stepDynamicParameters(time);
    }

    void GaitPlanner::stepDynamicParameters(scalar_t time)
    {
      if(targetDynamicParams_ == dynamicParams_)
      {
        lastStepTime_ = time;
        return;
      }

      // Parameters can be changed only after the last change
      if(time <= currentDynamicParamsTime_) return;

      const bool standing = isStanding(dynamicParams_.steppingFrequency);
      const bool targetStanding = isStanding(targetDynamicParams_.steppingFrequency);

      // Offsets are compared modulo 1 (e.g. -0.5 and 0.5 are the same gait)
      scalar_t phaseOffsetsDifference = 0.0;
      for(size_t i = 0; i < targetDynamicParams_.phaseOffsets.size(); ++i)
      {
        const scalar_t difference = normalizePhase(targetDynamicParams_.phaseOffsets[i] -
          dynamicParams_.phaseOffsets[i]);
        phaseOffsetsDifference += std::min(difference, 1.0 - difference);
      }
      const bool gaitTypeChange = phaseOffsetsDifference >= Definitions::GAIT_PARAMETERS_EPS;

      // Start, stop and gait type change can not be done step by step, new gait starts with new phase
      if(standing || targetStanding || gaitTypeChange)
      {
        RephasingMode mode = RephasingMode::WALKING;
        if(targetStanding)
        {
          mode = standing ? RephasingMode::STANDING : RephasingMode::STOP;
        }

        // No valid phase for standing robot -> change waits for start
        const auto newBasePhase = getRephasedBasePhase(time, targetDynamicParams_, mode);
        if(newBasePhase)
        {
          applyDynamicParameters(time, targetDynamicParams_, newBasePhase);
        }
        lastStepTime_ = time;
        return;
      }

      // Long pause between steps (e.g. swing ratio blocked by contacts) does not make a big step
      const scalar_t stepDuration = std::min(time - lastStepTime_, Definitions::MAX_PARAMETERS_STEP_DURATION);

      const auto stepTowards = [](scalar_t current, scalar_t target, scalar_t maximumRate, scalar_t duration)
      {
        // No limit
        if(maximumRate <= 0.0) return target;
        const scalar_t maximumStep = maximumRate * duration;
        if(std::abs(target - current) <= maximumStep) return target;
        return current + std::copysign(maximumStep, target - current);
      };

      // Walking with the same phase offsets (otherwise target was applied at once)
      GaitDynamicParameters newParams = targetDynamicParams_;

      newParams.steppingFrequency = stepTowards(dynamicParams_.steppingFrequency,
        targetDynamicParams_.steppingFrequency, staticParams_.maximumSteppingFrequencyRate, stepDuration);

      const scalar_t swingRatio = stepTowards(dynamicParams_.swingRatio, targetDynamicParams_.swingRatio,
        staticParams_.maximumSwingRatioRate, stepDuration);
      newParams.swingRatio = getContactPreservingSwingRatio(time, dynamicParams_.swingRatio,
        swingRatio, newParams.phaseOffsets);

      // No change (swing ratio is blocked by contacts or step is too small), step grows until next call
      if(newParams == dynamicParams_) return;

      applyDynamicParameters(time, newParams);
      lastStepTime_ = time;
    }

    bool GaitPlanner::isStanding(scalar_t steppingFrequency) const
    {
      // Same as in getDynamicModeSequenceTemplate()
      return steppingFrequency < staticParams_.minimumSteppingFrequency ||
        numerics::almost_eq(steppingFrequency, staticParams_.minimumSteppingFrequency, SCALAR_EPSILON);
    }

    scalar_t GaitPlanner::getContactPreservingSwingRatio(scalar_t time, scalar_t currentSwingRatio,
      scalar_t newSwingRatio, const std::vector<scalar_t>& phaseOffsets) const
    {
      // Phases of end effectors at time, same as in applyDynamicParameters()
      const scalar_t basePhase = phaseController_.getPhasesAtTime(time)[0];
      std::vector<scalar_t> phases(staticParams_.endEffectorNumber);
      phases[0] = normalizePhase(basePhase - SCALAR_EPSILON);
      for(size_t i = 1; i < staticParams_.endEffectorNumber; ++i)
      {
        phases[i] = normalizePhase(basePhase + phaseOffsets[i - 1] - SCALAR_EPSILON);
      }

      scalar_t swingRatio = newSwingRatio;

      if(newSwingRatio > currentSwingRatio)
      {
        // End effectors in contact (phase >= current swing ratio) below the new swing ratio would lift off,
        // swing ratio equal to the phase keeps the contact
        for(const scalar_t phase: phases)
        {
          if(phase >= currentSwingRatio && phase < swingRatio)
          {
            swingRatio = phase;
          }
        }
      }
      else if(newSwingRatio < currentSwingRatio)
      {
        // Swinging end effectors (phase < current swing ratio) above the new swing ratio would touch down
        scalar_t highestSwingPhase = -1.0;
        for(const scalar_t phase: phases)
        {
          if(phase < currentSwingRatio && phase >= newSwingRatio)
          {
            highestSwingPhase = std::max(highestSwingPhase, phase);
          }
        }

        if(highestSwingPhase >= 0.0)
        {
          swingRatio = std::min(currentSwingRatio, highestSwingPhase + Definitions::SWING_RATIO_MARGIN);
        }
      }

      return std::clamp(swingRatio, 0.0, 1.0);
    }

    std::optional<scalar_t> GaitPlanner::getRephasedBasePhase(scalar_t time,
      const GaitDynamicParameters& newParams, RephasingMode mode) const
    {
      const size_t endEffectorNumber = staticParams_.endEffectorNumber;
      const scalar_t swingRatio = newParams.swingRatio;
      const auto offset = [&newParams](size_t i)
      {
        return i == 0 ? 0.0 : newParams.phaseOffsets[i - 1];
      };

      // Planned contact states at time and phase without re-phasing
      const contact_flags_t currentFlags = phaseController_.getContactFlagsAtTime(time);
      const scalar_t naturalPhase = phaseController_.getPhasesAtTime(time)[0];

      // Base phases in which contact state of the new gait changes: lift off (phase 0) and touchdown
      // (phase equal to swing ratio) of every end effector. Contact states are constant between them.
      std::vector<scalar_t> breakpoints;
      breakpoints.reserve(2 * endEffectorNumber);
      for(size_t i = 0; i < endEffectorNumber; ++i)
      {
        breakpoints.push_back(normalizePhase(-offset(i)));
        breakpoints.push_back(normalizePhase(swingRatio - offset(i)));
      }
      std::sort(breakpoints.begin(), breakpoints.end());
      const scalar_t breakpointEps = 1e-9;
      breakpoints.erase(std::unique(breakpoints.begin(), breakpoints.end(),
        [breakpointEps](scalar_t lhs, scalar_t rhs) {return rhs - lhs < breakpointEps;}), breakpoints.end());
      if(breakpoints.size() > 1 && breakpoints.back() - breakpoints.front() > 1.0 - breakpointEps)
      {
        breakpoints.pop_back();
      }

      // Distance on phase circle
      const auto phaseDistance = [](scalar_t lhs, scalar_t rhs)
      {
        const scalar_t difference = normalizePhase(lhs - rhs);
        return std::min(difference, 1.0 - difference);
      };

      bool found = false;
      std::pair<int, int> bestKey;
      scalar_t bestDistance = 0.0;
      scalar_t bestPhase = 0.0;

      for(size_t k = 0; k < breakpoints.size(); ++k)
      {
        // Arc [start, end) of base phase with constant contact states
        const scalar_t arcStart = breakpoints[k];
        const scalar_t arcEnd = (k + 1 < breakpoints.size()) ? breakpoints[k + 1] : breakpoints.front() + 1.0;
        const scalar_t arcLength = arcEnd - arcStart;
        const scalar_t middlePhase = arcStart + 0.5 * arcLength;

        int changes = 0;
        int lifts = 0;
        int contacts = 0;
        for(size_t i = 0; i < endEffectorNumber; ++i)
        {
          const bool contact = normalizePhase(middlePhase + offset(i)) >= swingRatio;
          contacts += contact;
          changes += (contact != currentFlags[i]);
          lifts += (currentFlags[i] && !contact);
        }

        std::pair<int, int> key;
        switch(mode)
        {
          case RephasingMode::WALKING:
            key = {changes, -contacts};
            break;
          case RephasingMode::STANDING:
            if(lifts > 0) continue;
            key = {-contacts, changes};
            break;
          case RephasingMode::STOP:
            key = {-contacts, changes};
            break;
        }

        // Contact states just after time matter (new gait starts after time), natural phase on
        // a breakpoint (e.g. standing robot) belongs to the arc that starts there
        scalar_t phase;
        scalar_t naturalInArc = naturalPhase;
        if(naturalInArc < arcStart) naturalInArc += 1.0;

        // Phases do not change when standing, phase on a breakpoint would have different contact
        // states in contact flags (- SCALAR_EPSILON) and mode schedule
        const bool naturalOnBreakpoint = naturalInArc - arcStart < breakpointEps ||
          arcEnd - naturalInArc < breakpointEps;

        if(naturalInArc < arcEnd && (mode == RephasingMode::WALKING || !naturalOnBreakpoint))
        {
          // Contact states are the best ones without re-phasing, phase is continuous
          phase = naturalPhase;
        }
        else if(mode != RephasingMode::WALKING)
        {
          // Phases do not change when standing, middle is the safest
          phase = middlePhase;
        }
        else
        {
          const scalar_t margin = std::min(0.5 * arcLength,
            newParams.steppingFrequency * Definitions::REPHASING_MARGIN_TIME);

          if(changes == 0)
          {
            // Contact states are kept, phase is as close to the natural one as possible
            const scalar_t first = arcStart + margin;
            const scalar_t last = arcEnd - margin;
            phase = phaseDistance(first, naturalPhase) <= phaseDistance(last, naturalPhase) ? first : last;
          }
          else
          {
            // End effectors that change contact state are at the start of the new state
            phase = arcStart + margin;
          }
        }

        const scalar_t distance = phaseDistance(phase, naturalPhase);
        if(!found || key < bestKey || (key == bestKey && distance < bestDistance))
        {
          found = true;
          bestKey = key;
          bestDistance = distance;
          bestPhase = phase;
        }
      }

      if(!found) return std::nullopt;
      return normalizePhase(bestPhase);
    }

    void GaitPlanner::applyDynamicParameters(scalar_t time,
      const GaitDynamicParameters& clampedParams, std::optional<scalar_t> newBasePhase)
    {
      assert(time >= currentDynamicParamsTime_);

      // Compare clamped parameters, stored parameters are also clamped
      if(clampedParams == dynamicParams_) return;

      // Phase is continuous when there is no new one (or it is the same)
      const scalar_t naturalBasePhase = phaseController_.getPhasesAtTime(time)[0];
      if(newBasePhase)
      {
        const scalar_t difference = normalizePhase(*newBasePhase - naturalBasePhase);
        if(std::min(difference, 1.0 - difference) < SCALAR_EPSILON) newBasePhase.reset();
      }

      // Check mode with new parameters at time t
      const scalar_t basePhase = newBasePhase ? *newBasePhase : naturalBasePhase;
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
        if(newBasePhase)
        {
          phaseController_.update(currentDynamicParamsTime_, clampedParams, *newBasePhase);
        }
        else
        {
          phaseController_.update(currentDynamicParamsTime_, clampedParams);
        }

        // Event at the change time, mode after it is a placeholder: getModeSchedule() tiles from this event
        // with new parameters and moves it to the next mode change. Event at the next mode change would be
        // removed, when it is after the final time (e.g. infinite time of standing), then tiling with
        // new parameters would start before the change time.
        eventTimes.push_back(currentDynamicParamsTime_);
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

        if(newBasePhase)
        {
          phaseController_.update(currentDynamicParamsTime_, clampedParams, *newBasePhase);
        }
        else
        {
          phaseController_.update(currentDynamicParamsTime_, clampedParams);
        }

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

    const GaitDynamicParameters& GaitPlanner::getTargetDynamicParameters() const
    {
      return targetDynamicParams_;
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