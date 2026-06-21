#include <rclcpp/rclcpp.hpp>

#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include <legged_locomotion_mpc_ros2/controller/LeggedMpcController.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  using namespace legged_locomotion_mpc_ros2;

  auto leggedMpcController = std::make_shared<LeggedMpcController>();

  rclcpp::executors::MultiThreadedExecutor executor;

  executor.add_node(leggedMpcController->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();
  return 0;
}