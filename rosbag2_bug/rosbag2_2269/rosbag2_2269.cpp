#include <sstream>
#include <vector>
#include "rosbag2_transport/record_options.hpp"

int main(int argc, char ** argv)
{
  (void)argc;
  (void)argv;

  rosbag2_transport::RecordOptions options;
  options.topics.reserve(2);
  options.topics.push_back("topic");
  options.topics.push_back("other_topic");
  options.exclude_topics.reserve(2);
  options.exclude_topics.push_back("exclude_topic1");
  options.exclude_topics.push_back("exclude_topic2");

  YAML::Node node = YAML::convert<rosbag2_transport::RecordOptions>().encode(options);
  std::stringstream ss;
  ss << node;
  std::cout << "Serialized RecordOptions to YAML:\n" << ss.str() << std::endl;
  YAML::Node reconstructed_node = YAML::Load(ss.str());
  rosbag2_transport::RecordOptions reconstructed = reconstructed_node.as<rosbag2_transport::RecordOptions>();

  for (auto const & topic : reconstructed.topics) {
    std::cout << "Included topic: " << topic << std::endl;
  }
  for (auto const & topic : reconstructed.exclude_topics) {
    std::cout << "Excluded topic: " << topic << std::endl;
  }

  return 0;
}
