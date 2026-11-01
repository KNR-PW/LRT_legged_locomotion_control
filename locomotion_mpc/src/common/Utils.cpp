#include <legged_locomotion_mpc/common/Utils.hpp>

#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_core/misc/Numerics.h>


namespace legged_locomotion_mpc
{ 
  namespace utils
  {

    using namespace ocs2;
    using namespace floating_base_model;
    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    TargetTrajectories subsampleReferenceTrajectory(
      const TargetTrajectories& targetTrajectories,
      scalar_t initTime,
      scalar_t finalTime,
      scalar_t maximumReferenceSampleInterval)
    {
      // Need a copy to
      // 1. possibly overwrite joint references later (adapted with inverse kinematics)
      // 2. ensure a maximum interval between references points.
      // 3. unsure we have samples at start and end of the MPC horizon.
      if(targetTrajectories.empty()) 
      {
        throw std::runtime_error("[Utils]: provided target "
          "trajectory cannot be empty.");
      }

      TargetTrajectories newTargetTrajectories;
      // Add first reference
      {
        const auto initInterpIndex = LinearInterpolation::timeSegment(
          initTime, targetTrajectories.timeTrajectory);

        newTargetTrajectories.timeTrajectory.push_back(initTime);

        newTargetTrajectories.stateTrajectory.push_back(
          LinearInterpolation::interpolate(initInterpIndex, targetTrajectories.stateTrajectory));

        newTargetTrajectories.inputTrajectory.push_back(
          LinearInterpolation::interpolate(initInterpIndex, targetTrajectories.inputTrajectory));
      }
      bool finalTimeFlag = true;

      for (int k = 0; k < targetTrajectories.timeTrajectory.size(); ++k) 
      {
        if(targetTrajectories.timeTrajectory[k] <= initTime) 
        {
          continue; // Drop all samples before init time
        } 
        else if(targetTrajectories.timeTrajectory[k] > finalTime && finalTimeFlag) 
        {
          // Do it one time only!
          finalTimeFlag = false;

          // Add final time sample. Samples after final time are also kept for 
          // touchdowns after the horizon.
          const auto finalInterpIndex = LinearInterpolation::timeSegment(
            finalTime, targetTrajectories.timeTrajectory);

          newTargetTrajectories.timeTrajectory.push_back(finalTime);

          newTargetTrajectories.stateTrajectory.push_back(
            LinearInterpolation::interpolate(finalInterpIndex, targetTrajectories.stateTrajectory));

          newTargetTrajectories.inputTrajectory.push_back(
            LinearInterpolation::interpolate(finalInterpIndex, targetTrajectories.inputTrajectory));
        }

        // Check if we need to add extra intermediate samples
        while (newTargetTrajectories.timeTrajectory.back() + 
          maximumReferenceSampleInterval < targetTrajectories.timeTrajectory[k]) 
        {
          const scalar_t t = newTargetTrajectories.timeTrajectory.back() + maximumReferenceSampleInterval;

          const auto interpIndex = LinearInterpolation::timeSegment(t, targetTrajectories.timeTrajectory);

          newTargetTrajectories.timeTrajectory.push_back(t);

          newTargetTrajectories.stateTrajectory.push_back(
            LinearInterpolation::interpolate(interpIndex, targetTrajectories.stateTrajectory));

          newTargetTrajectories.inputTrajectory.push_back(
            LinearInterpolation::interpolate(interpIndex, targetTrajectories.inputTrajectory));
        }
        // Add the original reference sample
        newTargetTrajectories.timeTrajectory.push_back(targetTrajectories.timeTrajectory[k]);
        newTargetTrajectories.stateTrajectory.push_back(targetTrajectories.stateTrajectory[k]);
        newTargetTrajectories.inputTrajectory.push_back(targetTrajectories.inputTrajectory[k]);
      }
      return newTargetTrajectories;
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    size_t numberOfClosedContacts(const FloatingBaseModelInfo &info,
      const contact_flags_t &contactFlags) 
    {
      size_t numEndEffectors = info.numThreeDofContacts + info.numSixDofContacts;

      return contactFlags.count() > numEndEffectors ? numEndEffectors : contactFlags.count();
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    vector_t weightCompensatingInput(const FloatingBaseModelInfo &info, 
      const contact_flags_t &contactFlags)
    {
      const size_t numStanceLegs = numberOfClosedContacts(info, contactFlags);
      size_t numEndEffectors = info.numThreeDofContacts + info.numSixDofContacts;
      vector_t input = vector_t::Zero(info.inputDim);
      if(numStanceLegs > 0) 
      {
        const scalar_t totalWeight = info.robotMass * PLUS_GRAVITY_VALUE;
        const vector3_t forceInInertialFrame(0.0, 0.0, totalWeight / numStanceLegs);
        for (size_t i = 0; i < numEndEffectors; i++) 
        {
          if(contactFlags[i]) {
            access_helper_functions::getContactForces(input, i, info) = forceInInertialFrame;
          }
        } 
      }
      return input;
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    void weightCompensatingAppendInput(vector_t& input, const FloatingBaseModelInfo &info, 
      const contact_flags_t &contactFlags)
    {
      const auto numStanceLegs = numberOfClosedContacts(info, contactFlags);
      size_t numEndEffectors = info.numThreeDofContacts + info.numSixDofContacts;
      const scalar_t totalWeight = info.robotMass * PLUS_GRAVITY_VALUE;
      for (size_t i = 0; i < numEndEffectors; i++)
      {
        if(contactFlags[i])
        {
          // At least this leg is in contact, so numStanceLegs > 0
          access_helper_functions::getContactForces(input, i, info) =
            vector3_t(0.0, 0.0, totalWeight / numStanceLegs);
        }
        else
        {
          // Legs not in contact have no force, even if input had one before
          access_helper_functions::getContactForces(input, i, info).setZero();
        }
      }
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    size_t findIndexInTimeArray(const std::vector<scalar_t> &timeArray, scalar_t time) 
    {
      const auto compare = [](const scalar_t& one, const scalar_t& two)
      {
        return one < two && !numerics::almost_eq(one, two, SCALAR_EPSILON);
      };
      const auto firstLargerValueIterator = std::lower_bound(timeArray.cbegin(), timeArray.cend(), time, compare);
      return static_cast<size_t>(firstLargerValueIterator - timeArray.cbegin());
    }
    
  } // namsespace utils
} // namespace legged_locomotiom_mpc