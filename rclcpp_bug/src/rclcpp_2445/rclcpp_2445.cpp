#include <memory>
#include <thread>
#include <string>
#include <vector>

#include "rclcpp/executors.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/timer.hpp"
#include "std_msgs/msg/string.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("test_callback_group");
  auto sub = node->create_subscription<std_msgs::msg::String>(
      "/test_callback_group", 10,
      [](const std_msgs::msg::String::ConstSharedPtr) {});
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),
                                                    3u);
  std::thread thr;
  {
    auto group = node->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    executor.add_node(node);
    thr = std::thread([&executor]() { executor.spin(); });
    std::cout << "wait for executor running..." << std::endl;
  } // executor wait on dds and then destory the callback group

  while (rclcpp::ok()) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  rclcpp::shutdown();

  if (thr.joinable()) {
    thr.join();
  }

  rclcpp::shutdown();
  return 0;
} 
