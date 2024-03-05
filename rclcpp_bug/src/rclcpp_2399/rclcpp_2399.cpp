#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std_msgs::msg::Header header;

  header.stamp = rclcpp::Duration::from_seconds(1) + header.stamp;

  std::cout << header.stamp.sec << "." << header.stamp.nanosec << std::endl;

  rclcpp::shutdown();

  return 0;
}