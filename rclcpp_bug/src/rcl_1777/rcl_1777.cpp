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
  bool ret = rcl_parse_yaml_file(filename, params);
  if (!ret) {
    rcutils_error_string_t error_string = rcutils_get_error_string();
    fprintf(stderr, "%s\n", error_string.str);
    rcutils_reset_error();
  }

  rcl_yaml_node_struct_fini(params);

  rclcpp::shutdown();
  return 0;
}