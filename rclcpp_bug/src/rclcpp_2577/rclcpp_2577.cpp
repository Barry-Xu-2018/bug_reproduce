#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("vector_integer_parameters");

  // Declaring Params
  node->declare_parameter("param", std::vector<int>{});

  // Getting Params
  std::vector<int> clamps_iters = {1,2,3};
  node->get_parameter("param", clamps_iters);

  rclcpp::spin(node);
  rclcpp::shutdown();
}