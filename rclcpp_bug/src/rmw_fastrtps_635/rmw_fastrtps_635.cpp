#include <iostream>
#include <memory>
#include <string>
#include <chrono>

#include "rosgraph_msgs/msg/clock.hpp"
#include "std_msgs/msg/string.hpp"

#include "rclcpp/rclcpp.hpp"


class ClockNode : public rclcpp::Node
{
public:
  ClockNode()
  : rclcpp::Node("ClockNode")
  {

    rclcpp::QoS clock_qos = rclcpp::ClockQoS();
    pub_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", clock_qos);
    auto clock_callback = [this]() ->void {
      rosgraph_msgs::msg::Clock msg;
      msg.clock.sec = (count_ * 500)/1000;
      msg.clock.nanosec = count_ * 1000000;
      this->pub_->publish(msg);
      count_++;
    };

    timer_ = this->create_wall_timer(std::chrono::milliseconds(500), clock_callback);
  }
  ~ClockNode() {
    timer_->cancel();
  }
private:
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  uint64_t count_ = 0; // step is 500ms
};

class ForwardNode : public rclcpp::Node
{
public:
  ForwardNode()
  : rclcpp::Node("ForwardNode")
  {
    //this->set_parameter(rclcpp::Parameter("use_sim_time", true));
    rclcpp::QoS qos(rclcpp::KeepLast{10});
    pub_ = this->create_publisher<std_msgs::msg::String>("/forword_chatter", qos);

    auto forword = [this](std_msgs::msg::String::ConstSharedPtr msg)->void {
      std_msgs::msg::String forword_msg;
      forword_msg.set__data(msg->data + " forword");
      this->pub_->publish(forword_msg);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s' -- %lu", msg->data.c_str(), this->get_clock()->now().nanoseconds());
    };

    sub_ = this->create_subscription<std_msgs::msg::String>("/chatter", qos, forword);
  }
private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto clock_node = std::make_shared<ClockNode>();
  auto pub_node = std::make_shared<ForwardNode>();

  rclcpp::executors::SingleThreadedExecutor exec;

  exec.add_node(clock_node);
  exec.add_node(pub_node);

  exec.spin();

  rclcpp::shutdown();

  return 0;
}
