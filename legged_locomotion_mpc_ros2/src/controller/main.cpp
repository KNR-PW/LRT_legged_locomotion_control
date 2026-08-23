#include <rclcpp/rclcpp.hpp>

#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include <legged_locomotion_mpc_ros2/controller/LeggedLocomotionMpcControllerNode.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  using namespace legged_locomotion_mpc_ros2;

  auto leggedLocomotionMpcControllerNode = std::make_shared<LeggedLocomotionMpcControllerNode>();

  rclcpp::executors::MultiThreadedExecutor executor;

  executor.add_node(leggedLocomotionMpcControllerNode->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();
  return 0;
}