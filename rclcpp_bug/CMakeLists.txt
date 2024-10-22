cmake_minimum_required(VERSION 3.8)
project(bug_reproduce_cpp)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rcl REQUIRED)                                                                                                                                                                               
find_package(rclcpp REQUIRED)                                                                                                                                                                               
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(example_interfaces REQUIRED)
#find_package(rosidl_default_generators REQUIRED)
find_package(bug_reproduce_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)
#find_package(action_tutorials_interfaces REQUIRED)

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

#add_executable(rclcpp_1785 src/rclcpp_1785/rclcpp_1785.cpp)
#ament_target_dependencies(rclcpp_1785 rclcpp example_interfaces)

add_executable(rclcpp_1809 src/rclcpp_1809/rclcpp_1809.cpp)
ament_target_dependencies(rclcpp_1809 rclcpp)

add_executable(rclcpp_1983 src/rclcpp_1983/rclcpp_1983.cpp)
ament_target_dependencies(rclcpp_1983 rclcpp)

add_executable(rclcpp_2012 src/rclcpp_2012/rclcpp_2012.cpp)
ament_target_dependencies(rclcpp_2012 rclcpp std_msgs)

add_executable(rclcpp_2154 src/rclcpp_2154/rclcpp_2154.cpp)
ament_target_dependencies(rclcpp_2154 rclcpp rcl_interfaces std_msgs)

add_executable(rclcpp_2399 src/rclcpp_2399/rclcpp_2399.cpp)
ament_target_dependencies(rclcpp_2399 rclcpp rcl_interfaces std_msgs)

add_executable(rclcpp_2445 src/rclcpp_2445/rclcpp_2445.cpp)
ament_target_dependencies(rclcpp_2445 rclcpp std_msgs)

add_executable(rclcpp_2446_pub src/rclcpp_2446/pub.cpp)
ament_target_dependencies(rclcpp_2446_pub rclcpp std_msgs bug_reproduce_interfaces)

add_executable(rclcpp_2446_sub src/rclcpp_2446/sub.cpp)
ament_target_dependencies(rclcpp_2446_sub rclcpp std_msgs bug_reproduce_interfaces)

add_executable(rmw_fastrtps_635 src/rmw_fastrtps_635/rmw_fastrtps_635.cpp)
ament_target_dependencies(rmw_fastrtps_635 rclcpp builtin_interfaces std_msgs)

add_executable(rmw_fastrtps_681 src/rmw_fastrtps_681/rmw_fastrtps_681.cpp)
ament_target_dependencies(rmw_fastrtps_681 rclcpp std_msgs)

add_executable(rclcpp_2577 src/rclcpp_2577/rclcpp_2577.cpp)
ament_target_dependencies(rclcpp_2577 rclcpp)

add_executable(rclcpp_2101 src/rclcpp_2101/rclcpp_2101.cpp)
ament_target_dependencies(rclcpp_2101 rclcpp_action example_interfaces)

add_executable(rclcpp_2655 src/rclcpp_2655/rclcpp_2655.cpp)
ament_target_dependencies(rclcpp_2655 rclcpp std_msgs)

install(TARGETS
  data_sharing_listener
  data_sharing_talker
  rclcpp_1694
	rclcpp_1691
  rclcpp_1749
  #rclcpp_1785
  rclcpp_1809
  rclcpp_1983
  rclcpp_2012
  rclcpp_2154
  rclcpp_2399
  rclcpp_2445
  rclcpp_2446_pub
  rclcpp_2446_sub
  rmw_fastrtps_635
  rmw_fastrtps_681
  rclcpp_2577
  rclcpp_2101
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
