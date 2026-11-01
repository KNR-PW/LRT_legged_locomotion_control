#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

#include <legged_state_estimator_ros2/LeggedStateEstimatorNode.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  using namespace legged_state_estimator_ros2;

  auto leggedStateEstimatorNode = std::make_shared<LeggedStateEstimatorNode>();

  // All callbacks are in one mutually exclusive callback group, more threads would only wait
  rclcpp::executors::SingleThreadedExecutor executor;

  executor.add_node(leggedStateEstimatorNode->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();
  return 0;
}