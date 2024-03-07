// pub.cpp
#include <chrono>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "ros_adapter.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
      : Node("minimal_publisher"), count_(0)
  {
    auto timer_callback =
        [this]() -> void
    {
#ifdef ROS_TYPE
      MsgT msg;
      msg.id = this->count_++;
      msg.time = 0x12345678;
#else
      MsgT msg = "my name!";
#endif
      this->publisher_->publish(msg);
    };

    publisher_ = this->create_publisher<MsgT>("topic", 10);
    timer_ = create_wall_timer(30ms, timer_callback);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<MsgT>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
