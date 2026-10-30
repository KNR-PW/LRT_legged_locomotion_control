#include <rclcpp/rclcpp.hpp>

#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include <legged_whole_body_control_ros2/LeggedWholeBodyControllerNode.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  using namespace legged_whole_body_control_ros2;

  auto leggedWholeBodyControllerNode = std::make_shared<LeggedWholeBodyControllerNode>();

  rclcpp::executors::MultiThreadedExecutor executor;

  executor.add_node(leggedWholeBodyControllerNode->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();
  return 0;
}