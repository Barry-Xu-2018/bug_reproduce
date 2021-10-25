#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("my_node", "ns");
  RCLCPP_INFO(node->get_logger(), "namespace: %s", node->get_effective_namespace().c_str());


  auto sub_node = node->create_sub_node("sub_ns");
  RCLCPP_INFO(sub_node->get_logger(), "namespace: %s", sub_node->get_effective_namespace().c_str());

  rclcpp::shutdown();

  return 0;
}
