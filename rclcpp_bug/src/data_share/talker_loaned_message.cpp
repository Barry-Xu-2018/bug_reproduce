// Copyright 2019 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <cstdio>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

// Create a Talker class that subclasses the generic rclcpp::Node base class.
// The main function below will instantiate the class as a ROS node.
class LoanedMessageTalker : public rclcpp::Node
{
public:
  LoanedMessageTalker()
  : Node("loaned_message_talker")
  {
    // Create a function for when messages are to be sent.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // We differentiate in this demo between two fundamental message types - POD and non-POD
    // PODs are plain old data types, meaning all the data of its type is encapsulated within
    // the structure and does not require any heap allocation or dynamic resizing.
    // non-PODs are essentially the opposite where the data size changes during runtime.
    // All containers (including Strings) are such non-PODs.
    // Most middlewares won't be able to loan non-POD datatypes.
    // We thus feature two publishers in this demo where both, a POD and non-POD message
    // will be used to publish data.
    // The take-away for this is that the rclcpp API for message loaning can cope with
    // either POD and non-POD transparently.
    auto publish_message =
      [this]() -> void
      {
        count_++;
        // We loan a message here and don't allocate the memory on the stack.
        // For middlewares which support message loaning, this means the middleware
        // completely owns the memory for this message.
        // This enables a zero-copy message transport for middlewares with shared memory
        // capabilities.
        // If the middleware doesn't support this, the loaned message will be allocated
        // with the allocator instance provided by the publisher.
        auto loaned_msg = pod_pub_->borrow_loaned_message();
        auto pod_msg_data = static_cast<double>(count_);
        loaned_msg.get().data = pod_msg_data;

        RCLCPP_INFO(this->get_logger(), "Publishing: '%f' --- %p", pod_msg_data, static_cast<void *>(&(loaned_msg.get().data)));
        // As the middleware might own the memory allocated for this message,
        // a call to publish explicitly transfers ownership back to the middleware.
        // The loaned message instance is thus no longer valid after a call to publish.
        //pod_pub_->publish(std::move(pod_loaned_msg));
        pod_pub_->publish(std::move(loaned_msg));

      };

    // Create a publisher with a custom Quality of Service profile.
    rclcpp::QoS qos(rclcpp::KeepLast(7));
    pod_pub_ = this->create_publisher<std_msgs::msg::Float64>("chatter_pod", qos);

    // Use a timer to schedule periodic message publishing.
    timer_ = this->create_wall_timer(1s, publish_message);
  }

private:
  size_t count_ = 1;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pod_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LoanedMessageTalker>());
  rclcpp::shutdown();
  return 0;
}
