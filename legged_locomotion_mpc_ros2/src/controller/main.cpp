#include <rclcpp/rclcpp.hpp>

#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include <legged_locomotion_mpc_ros2/controller/LeggedLocomotionMpcController.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  using namespace legged_locomotion_mpc_ros2;

  auto leggedLocomotionMpcController = std::make_shared<LeggedLocomotionMpcController>();

  rclcpp::executors::MultiThreadedExecutor executor;

  executor.add_node(leggedLocomotionMpcController->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();
  return 0;
}