// Copyright 2023 Sony Group Corporation.
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
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MatchedEventDetectNode : public rclcpp::Node
{
public:
  explicit MatchedEventDetectNode(
    const std::string & pub_topic_name)
  : Node("matched_event_detect_node")
  {
    rclcpp::PublisherOptions pub_options;
    pub_options.event_callbacks.matched_callback =
      [this](rclcpp::MatchedInfo & s) {
        RCLCPP_INFO(
          this->get_logger(),
          "Matched Event: %lu, %lu, %lu, %d", s.total_count, s.total_count_change, s.current_count, s.current_count_change);
      };

    pub_ = create_publisher<std_msgs::msg::String>(
      pub_topic_name, 10, pub_options);
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  std::shared_ptr<std::promise<bool>> promise_;
};

class MultiSubNode : public rclcpp::Node
{
public:
  explicit MultiSubNode(const std::string & topic_name)
  : Node("multi_sub_node"),
    topic_name_(topic_name)
  {}

  rclcpp::Subscription<std_msgs::msg::String>::WeakPtr create_one_sub(void)
  {
    RCLCPP_INFO(this->get_logger(), "Create a new subscription.");
    auto sub = create_subscription<std_msgs::msg::String>(
      topic_name_,
      10,
      [](std_msgs::msg::String::ConstSharedPtr) {});

    subs_.emplace_back(sub);
    return sub;
  }

private:
  std::string topic_name_;
  std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> subs_;
};

int main(int argc, char ** argv)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  std::string topic_name_for_detect_pub_matched_event = "pub_topic_matched_event_detect";

  rclcpp::executors::SingleThreadedExecutor executor;

  auto matched_event_detect_node = std::make_shared<MatchedEventDetectNode>(
    topic_name_for_detect_pub_matched_event);

  auto multi_sub_node = std::make_shared<MultiSubNode>(
    topic_name_for_detect_pub_matched_event);

  auto maximum_wait_time = 10s;

  executor.add_node(matched_event_detect_node);
  executor.add_node(multi_sub_node);

  // MatchedEventDetectNode will output:
  // First subscription is connected.
  auto sub1 = multi_sub_node->create_one_sub();
  executor.spin_all(maximum_wait_time);

  rclcpp::shutdown();
  return 0;
}
