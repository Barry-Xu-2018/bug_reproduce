#include <iostream>
#include <numeric>
#include <string>
#include <vector>

#include <rosbag2_transport/recorder.hpp>

namespace
{
rosidl_type_hash_t create_type_hash(int version, const std::vector<int>& values) {
  rosidl_type_hash_t type_hash;
  type_hash.version = version;
  for (size_t i = 0; i < ROSIDL_TYPE_HASH_SIZE && i < values.size(); ++i) {
    type_hash.value[i] = values[i];
  }
  return type_hash;
}

#if 0
rosidl_type_hash_t create_zero_hash() {
  return create_type_hash(0, std::vector<int>(ROSIDL_TYPE_HASH_SIZE, 0));
}
#endif

rosidl_type_hash_t create_v1_hash() {
  std::vector<int> values(ROSIDL_TYPE_HASH_SIZE);
  std::iota(values.begin(), values.end(), 1);
  return create_type_hash(1, values);
}

std::vector<rclcpp::TopicEndpointInfo> setup_endpoints(const std::vector<rosidl_type_hash_t>& hashes) {
  std::vector<rclcpp::TopicEndpointInfo> endpoints;
  for (const auto& hash : hashes) {
    rcl_topic_endpoint_info_t info;
    info.topic_type_hash = hash;
    info.topic_type = "test_topic_type";
    info.node_name = "test_node";
    info.node_namespace = "test_namespace";
    info.endpoint_type = RMW_ENDPOINT_INVALID;
    info.qos_profile = rmw_qos_profile_default;
    endpoints.emplace_back(rclcpp::TopicEndpointInfo(info));
  }
  return endpoints;
}
}

int main(int argc, char ** argv)
{
  (void)argc;
  (void)argv;

  auto endpoints = setup_endpoints({create_v1_hash(), create_v1_hash()});
  std::string result = rosbag2_transport::type_description_hash_for_topic(endpoints);
  std::cout << "Type description hash: " << result << std::endl;

  return 0;
}
