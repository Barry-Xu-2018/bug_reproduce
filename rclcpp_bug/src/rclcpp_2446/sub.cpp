// sub.cpp
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include "ros_adapter.hpp"

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
      : Node("minimal_subscriber")
  {
    auto callback = [this](const MsgT &msg)
    {
#ifdef ROS_TYPE
      std::cout << "msg.id = " << msg.id << std::endl;
      std::cout << "msg.time = " << msg.time << std::endl;
#else
      std::cout << "msg = " << msg << std::endl;
#endif
    };

    subscription_ = this->create_subscription<MsgT>("topic", 10, callback);
  }

private:
  rclcpp::Subscription<MsgT>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
