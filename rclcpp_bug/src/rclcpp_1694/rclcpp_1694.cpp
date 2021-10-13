#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging_macros.h"

class MyNode : public rclcpp::Node
{
public:
  explicit MyNode(const std::string & node_name) : rclcpp::Node(node_name)
  {
    RCLCPP_INFO(get_logger(), "Node logger");
    RCLCPP_INFO(get_logger().get_child("child"), "Child logger");
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exe;
  std::shared_ptr<MyNode> lc_node = std::make_shared<MyNode>("my_node");
  exe.add_node(lc_node->get_node_base_interface());
  exe.spin();

  rclcpp::shutdown();
  return 0;
}
