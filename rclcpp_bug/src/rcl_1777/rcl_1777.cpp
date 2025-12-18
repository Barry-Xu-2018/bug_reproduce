#include <stdlib.h>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rcl_params_t * params = rcl_yaml_node_struct_init(allocator);
  if (!params) {
    return 0;
  }

  char filename[] = "/tmp/test_params.yaml";
  (void)rcl_parse_yaml_file(filename, params);

  rcl_yaml_node_struct_fini(params);

  rclcpp::shutdown();
  return 0;
}