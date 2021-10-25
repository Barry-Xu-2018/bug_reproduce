cmake_minimum_required(VERSION 3.8)
project(bug_reproduce_rclcpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)                                                                                                                                                                               
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(example_interfaces REQUIRED)

add_executable(data_sharing_listener src/data_share/listener.cpp)
ament_target_dependencies(data_sharing_listener rclcpp std_msgs)

add_executable(data_sharing_talker src/data_share/talker_loaned_message.cpp)
ament_target_dependencies(data_sharing_talker rclcpp std_msgs)

add_executable(rclcpp_1694 src/rclcpp_1694/rclcpp_1694.cpp)
ament_target_dependencies(rclcpp_1694 rclcpp)

add_executable(rclcpp_1691 src/rclcpp_1691/rclcpp_1691.cpp)
ament_target_dependencies(rclcpp_1691 rclcpp)

add_executable(rclcpp_1749 src/rclcpp_1749/test_params_node.cpp)
ament_target_dependencies(rclcpp_1749 rclcpp)

add_executable(rclcpp_1785 src/rclcpp_1785/rclcpp_1785.cpp)
ament_target_dependencies(rclcpp_1785 rclcpp example_interfaces)

add_executable(rclcpp_1809 src/rclcpp_1809/rclcpp_1809.cpp)
ament_target_dependencies(rclcpp_1809 rclcpp)

install(TARGETS
  data_sharing_listener
  data_sharing_talker
  rclcpp_1694
	rclcpp_1691
  rclcpp_1749
  rclcpp_1785
  rclcpp_1809
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
