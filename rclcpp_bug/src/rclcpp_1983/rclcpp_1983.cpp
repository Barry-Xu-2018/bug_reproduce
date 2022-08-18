#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("node_1983");

  std::string calib;
  try {
      node->declare_parameter<std::string>("calib", "no");
      node->get_parameter_or("calib", calib, std::string("no"));
  }
  catch (std::exception e) {
      std::cout << e.what() << "\n";
  }
  std::cout << calib << "\n";

  rclcpp::shutdown();

  return 0;
}
