#include <rclcpp/node.hpp>
#include <rclcpp/executors.hpp>
#include <std_msgs/msg/string.hpp>

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::Node::SharedPtr pub_node = std::make_shared<rclcpp::Node>("publisher");
  auto pub = pub_node->create_publisher<std_msgs::msg::String>("topic", 100);
  int count = 0;
  auto timer_pub = pub_node->create_wall_timer(std::chrono::milliseconds(100), [&]() -> void {
    std_msgs::msg::String msg;
    msg.data = std::to_string(count);
    count++;
    pub->publish(msg);
    std::cout << "Published: " << msg.data << std::endl;
  });
  std::thread pub_node_thread([&]() {
    rclcpp::spin(pub_node);
  });

  rclcpp::Node::SharedPtr sub_node = std::make_shared<rclcpp::Node>("subscriber");
  auto sub =
      sub_node->create_subscription<std_msgs::msg::String>("topic", 100, [](std_msgs::msg::String::ConstSharedPtr msg) {
        std::cout << "Received: " << msg->data << std::endl;
      });

  while (rclcpp::ok()) {
    std::cout << "Running spin_some" << std::endl;
    rclcpp::spin_some(sub_node);
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  pub_node_thread.join();
  rclcpp::shutdown();

  return 0;
}