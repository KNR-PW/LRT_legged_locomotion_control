#include <legged_locomotion_mpc_ros2/controller/MpcPolicyConversions.hpp>

#include <limits>
#include <stdexcept>

#include <ocs2_ros_interfaces/common/RosMsgConversions.h>

namespace legged_locomotion_mpc_ros2
{
  using namespace ocs2;

  void createMpcPolicyMessage(const PrimalSolution& policy, const CommandData& command,
    const PerformanceIndex& performanceIndices, ocs2_msgs::msg::MpcFlattenedController& message)
  {
    const size_t pointNum = policy.timeTrajectory_.size();

    if(pointNum == 0 || policy.stateTrajectory_.size() != pointNum ||
       policy.inputTrajectory_.size() != pointNum)
    {
      throw std::runtime_error("[createMpcPolicyMessage]: Policy trajectories have different sizes!");
    }

    // Post event indices are sent as uint16
    if(pointNum > std::numeric_limits<uint16_t>::max())
    {
      throw std::runtime_error("[createMpcPolicyMessage]: Policy has too many time points!");
    }

    message.controller_type = ocs2_msgs::msg::MpcFlattenedController::CONTROLLER_FEEDFORWARD;

    message.init_observation = ros_msg_conversions::createObservationMsg(
      command.mpcInitObservation_);

    // Not used by the whole body controller, it would double the message size
    message.plan_target_trajectories.time_trajectory.clear();
    message.plan_target_trajectories.state_trajectory.clear();
    message.plan_target_trajectories.input_trajectory.clear();

    message.mode_schedule = ros_msg_conversions::createModeScheduleMsg(policy.modeSchedule_);

    message.performance_indices = ros_msg_conversions::createPerformanceIndicesMsg(
      command.mpcInitObservation_.time, performanceIndices);

    message.time_trajectory.assign(policy.timeTrajectory_.cbegin(), policy.timeTrajectory_.cend());

    message.post_event_indices.resize(policy.postEventIndices_.size());
    for(size_t i = 0; i < policy.postEventIndices_.size(); ++i)
    {
      message.post_event_indices[i] = static_cast<uint16_t>(policy.postEventIndices_[i]);
    }

    message.state_trajectory.resize(pointNum);
    message.input_trajectory.resize(pointNum);
    message.data.resize(pointNum);

    for(size_t k = 0; k < pointNum; ++k)
    {
      const vector_t& state = policy.stateTrajectory_[k];
      const vector_t& input = policy.inputTrajectory_[k];

      auto& stateValue = message.state_trajectory[k].value;
      stateValue.resize(state.size());
      Eigen::Map<Eigen::VectorXf>(stateValue.data(), state.size()) = state.cast<float>();

      auto& inputValue = message.input_trajectory[k].value;
      inputValue.resize(input.size());
      Eigen::Map<Eigen::VectorXf>(inputValue.data(), input.size()) = input.cast<float>();

      // Flattened feedforward controller is the input at time point (FeedforwardController::flatten)
      message.data[k].data = inputValue;
    }
  }
} // namespace legged_locomotion_mpc_ros2
