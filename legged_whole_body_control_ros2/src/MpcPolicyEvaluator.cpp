#include <legged_whole_body_control_ros2/MpcPolicyEvaluator.hpp>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <memory>
#include <stdexcept>

#include <ocs2_core/control/FeedforwardController.h>

#include <ocs2_ros_interfaces/common/RosMsgConversions.h>

namespace legged_whole_body_control_ros2
{
  using namespace ocs2;
  using namespace legged_whole_body_control;

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  MpcPolicyEvaluator::MpcPolicyEvaluator(size_t stateDim, size_t inputDim, size_t endEffectorNum):
    stateDim_(stateDim), inputDim_(inputDim), endEffectorNum_(endEffectorNum)
  {
    if(stateDim_ == 0 || inputDim_ == 0)
    {
      throw std::invalid_argument("[MpcPolicyEvaluator]: State and input dimensions have to be positive!");
    }

    // Modes are converted to contact flags
    if(endEffectorNum_ == 0 || endEffectorNum_ > MAX_LEG_NUMBER)
    {
      throw std::invalid_argument("[MpcPolicyEvaluator]: Number of end effectors has to be in [1, "
        + std::to_string(MAX_LEG_NUMBER) + "]!");
    }
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  bool MpcPolicyEvaluator::readPolicy(const legged_locomotion_msgs::msg::MpcPolicy& message,
    scalar_t timeOffset, std::string& error)
  {
    const auto& policyMessage = message.policy;

    if(!std::isfinite(timeOffset))
    {
      error = "time offset is not finite";
      return false;
    }

    if(policyMessage.controller_type != ocs2_msgs::msg::MpcFlattenedController::CONTROLLER_FEEDFORWARD)
    {
      error = "only feedforward controller is supported";
      return false;
    }

    const size_t pointNum = policyMessage.time_trajectory.size();

    if(pointNum == 0)
    {
      error = "policy is empty";
      return false;
    }

    if(policyMessage.state_trajectory.size() != pointNum ||
       policyMessage.input_trajectory.size() != pointNum || policyMessage.data.size() != pointNum)
    {
      error = "policy trajectories have different lengths";
      return false;
    }

    if(message.normals.size() != pointNum * endEffectorNum_)
    {
      error = "wrong number of terrain normals";
      return false;
    }

    // Time trajectory has to be sorted for interpolation
    for(size_t k = 0; k < pointNum; ++k)
    {
      const scalar_t time = policyMessage.time_trajectory[k];
      if(!std::isfinite(time) || (k > 0 && time < policyMessage.time_trajectory[k - 1]))
      {
        error = "time trajectory is not finite or not sorted";
        return false;
      }
    }

    for(const auto index: policyMessage.post_event_indices)
    {
      if(index >= pointNum)
      {
        error = "post event index outside of the time trajectory";
        return false;
      }
    }

    const auto& eventTimes = policyMessage.mode_schedule.event_times;
    const auto& modeSequence = policyMessage.mode_schedule.mode_sequence;

    if(modeSequence.size() != eventTimes.size() + 1)
    {
      error = "mode sequence size is not number of event times + 1";
      return false;
    }

    for(size_t i = 0; i < eventTimes.size(); ++i)
    {
      if(!std::isfinite(eventTimes[i]) || (i > 0 && eventTimes[i] < eventTimes[i - 1]))
      {
        error = "event times are not finite or not sorted";
        return false;
      }
    }

    // Mode is a bit set of contact flags, only end effector bits can be set
    const size_t modeNum = size_t(1) << endEffectorNum_;
    for(const auto mode: modeSequence)
    {
      if(mode < 0 || static_cast<size_t>(mode) >= modeNum)
      {
        error = "mode is not valid for this number of end effectors";
        return false;
      }
    }

    // New policy, active one in MRT is kept until the message is valid
    auto policy = std::make_unique<PrimalSolution>();

    policy->timeTrajectory_.resize(pointNum);
    policy->stateTrajectory_.resize(pointNum);
    policy->inputTrajectory_.resize(pointNum);

    std::vector<std::vector<float> const*> controllerData(pointNum);

    for(size_t k = 0; k < pointNum; ++k)
    {
      const auto& stateValue = policyMessage.state_trajectory[k].value;
      const auto& inputValue = policyMessage.input_trajectory[k].value;
      const auto& dataValue = policyMessage.data[k].data;

      if(stateValue.size() != stateDim_ || inputValue.size() != inputDim_ ||
         dataValue.size() != inputDim_)
      {
        error = "wrong state, input or controller dimension";
        return false;
      }

      const Eigen::Map<const Eigen::VectorXf> stateMap(stateValue.data(), stateValue.size());
      const Eigen::Map<const Eigen::VectorXf> inputMap(inputValue.data(), inputValue.size());
      const Eigen::Map<const Eigen::VectorXf> dataMap(dataValue.data(), dataValue.size());

      // Non-finite values would make every WBC problem unsolvable
      if(!stateMap.allFinite() || !inputMap.allFinite() || !dataMap.allFinite())
      {
        error = "state, input or controller has non-finite values";
        return false;
      }

      policy->timeTrajectory_[k] = policyMessage.time_trajectory[k] + timeOffset;
      policy->stateTrajectory_[k] = stateMap.cast<scalar_t>();
      policy->inputTrajectory_[k] = inputMap.cast<scalar_t>();
      controllerData[k] = &dataValue;
    }

    newTerrainNormals_.resize(pointNum);

    for(size_t k = 0; k < pointNum; ++k)
    {
      auto& normals = newTerrainNormals_[k];
      normals.resize(endEffectorNum_);

      for(size_t i = 0; i < endEffectorNum_; ++i)
      {
        const auto& normalMessage = message.normals[k * endEffectorNum_ + i];
        normals[i] = vector3_t(normalMessage.x, normalMessage.y, normalMessage.z);

        // WBC requires finite, non-zero normals (only asserted there)
        if(!normals[i].allFinite() || normals[i].norm() < 1e-6)
        {
          error = "terrain normal is zero or not finite";
          return false;
        }
      }
    }

    policy->postEventIndices_.assign(policyMessage.post_event_indices.cbegin(),
      policyMessage.post_event_indices.cend());

    policy->modeSchedule_ = ros_msg_conversions::readModeScheduleMsg(policyMessage.mode_schedule);
    for(auto& eventTime: policy->modeSchedule_.eventTimes)
    {
      eventTime += timeOffset;
    }

    policy->controllerPtr_ = std::make_unique<FeedforwardController>(
      FeedforwardController::unFlatten(policy->timeTrajectory_, controllerData));

    // Command and performance indices are not used by WBC, they are kept as in ocs2 MRT.
    // Target trajectories are not sent by MPC node (readTargetTrajectoriesMsg throws for empty ones)
    auto command = std::make_unique<CommandData>();

    command->mpcInitObservation_ = ros_msg_conversions::readObservationMsg(
      policyMessage.init_observation);
    command->mpcInitObservation_.time += timeOffset;

    auto performanceIndices = std::make_unique<PerformanceIndex>(
      ros_msg_conversions::readPerformanceIndicesMsg(policyMessage.performance_indices));

    mrt_.setNewPolicy(std::move(command), std::move(policy), std::move(performanceIndices));

    // Evaluator is used by one thread, so MRT buffer is never locked and the new policy
    // becomes active at once. Otherwise it would stay in the buffer until the next message
    if(!mrt_.updatePolicy())
    {
      error = "MRT policy buffer is locked";
      return false;
    }

    terrainNormals_.swap(newTerrainNormals_);
    hasPolicy_ = true;

    return true;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void MpcPolicyEvaluator::reset()
  {
    mrt_.reset();
    hasPolicy_ = false;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  scalar_t MpcPolicyEvaluator::getStartTime() const
  {
    assert(hasPolicy_);
    return mrt_.getPolicy().timeTrajectory_.front();
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  scalar_t MpcPolicyEvaluator::getFinalTime() const
  {
    assert(hasPolicy_);
    return mrt_.getPolicy().timeTrajectory_.back();
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void MpcPolicyEvaluator::evaluate(scalar_t time, const vector_t& measuredState,
    vector_t& state, vector_t& input)
  {
    assert(hasPolicy_);

    size_t mode = 0;
    mrt_.evaluatePolicy(time, measuredState, state, input, mode);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  contact_flags_t MpcPolicyEvaluator::getContactFlags(scalar_t time) const
  {
    assert(hasPolicy_);

    // Modes are checked in readPolicy, only end effector bits are set
    return contact_flags_t(mrt_.getPolicy().modeSchedule_.modeAtTime(time));
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  const std::vector<vector3_t>& MpcPolicyEvaluator::getTerrainNormals(scalar_t time) const
  {
    assert(hasPolicy_);
    return terrainNormals_[findTimePointIndex(time)];
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  size_t MpcPolicyEvaluator::findTimePointIndex(scalar_t time) const
  {
    const auto& times = mrt_.getPolicy().timeTrajectory_;
    const auto firstLaterTime = std::upper_bound(times.cbegin(), times.cend(), time);

    if(firstLaterTime == times.cbegin())
    {
      return 0;
    }

    return static_cast<size_t>(firstLaterTime - times.cbegin()) - 1;
  }
} // namespace legged_whole_body_control_ros2
