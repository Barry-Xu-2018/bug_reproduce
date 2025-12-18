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
target_link_libraries(data_sharing_listener PUBLIC rclcpp::rclcpp ${std_msgs_TARGETS})

add_executable(data_sharing_talker src/data_share/talker_loaned_message.cpp)
target_link_libraries(data_sharing_talker PUBLIC rclcpp::rclcpp ${std_msgs_TARGETS})

add_executable(rclcpp_1694 src/rclcpp_1694/rclcpp_1694.cpp)
target_link_libraries(rclcpp_1694 PUBLIC rclcpp::rclcpp)

add_executable(rclcpp_1691 src/rclcpp_1691/rclcpp_1691.cpp)
target_link_libraries(rclcpp_1691 PUBLIC rclcpp::rclcpp)

add_executable(rclcpp_1749 src/rclcpp_1749/test_params_node.cpp)
target_link_libraries(rclcpp_1749 PUBLIC rclcpp::rclcpp)

#add_executable(rclcpp_1785 src/rclcpp_1785/rclcpp_1785.cpp)
#target_link_libraries(rclcpp_1785 PUBLIC rclcpp::rclcpp ${example_interfaces_TARGETS})

add_executable(rclcpp_1809 src/rclcpp_1809/rclcpp_1809.cpp)
target_link_libraries(rclcpp_1809 PUBLIC rclcpp::rclcpp)

add_executable(rclcpp_1983 src/rclcpp_1983/rclcpp_1983.cpp)
target_link_libraries(rclcpp_1983 PUBLIC rclcpp::rclcpp)

add_executable(rclcpp_2012 src/rclcpp_2012/rclcpp_2012.cpp)
target_link_libraries(rclcpp_2012 PUBLIC rclcpp::rclcpp ${std_msgs_TARGETS})

add_executable(rclcpp_2154 src/rclcpp_2154/rclcpp_2154.cpp)
target_link_libraries(rclcpp_2154 PUBLIC rclcpp::rclcpp ${rcl_interfaces_TARGETS} ${std_msgs_TARGETS})

add_executable(rclcpp_2399 src/rclcpp_2399/rclcpp_2399.cpp)
target_link_libraries(rclcpp_2399 PUBLIC rclcpp::rclcpp ${rcl_interfaces_TARGETS} ${std_msgs_TARGETS})

add_executable(rclcpp_2445 src/rclcpp_2445/rclcpp_2445.cpp)
target_link_libraries(rclcpp_2445 PUBLIC rclcpp::rclcpp ${std_msgs_TARGETS})

add_executable(rclcpp_2446_pub src/rclcpp_2446/pub.cpp)
target_link_libraries(rclcpp_2446_pub PUBLIC rclcpp::rclcpp ${std_msgs_TARGETS} ${bug_reproduce_interfaces_TARGETS})

add_executable(rclcpp_2446_sub src/rclcpp_2446/sub.cpp)
target_link_libraries(rclcpp_2446_sub PUBLIC rclcpp::rclcpp ${std_msgs_TARGETS} ${bug_reproduce_interfaces_TARGETS})

add_executable(rmw_fastrtps_635 src/rmw_fastrtps_635/rmw_fastrtps_635.cpp)
target_link_libraries(rmw_fastrtps_635 PUBLIC rclcpp::rclcpp ${builtin_interfaces_TARGETS} ${std_msgs_TARGETS})

add_executable(rmw_fastrtps_681 src/rmw_fastrtps_681/rmw_fastrtps_681.cpp)
target_link_libraries(rmw_fastrtps_681 PUBLIC rclcpp::rclcpp ${std_msgs_TARGETS})

add_executable(rclcpp_2577 src/rclcpp_2577/rclcpp_2577.cpp)
target_link_libraries(rclcpp_2577 PUBLIC rclcpp::rclcpp)

add_executable(rclcpp_2101 src/rclcpp_2101/rclcpp_2101.cpp)
target_link_libraries(rclcpp_2101 PUBLIC rclcpp_action::rclcpp_action ${example_interfaces_TARGETS})

add_executable(rclcpp_2655 src/rclcpp_2655/rclcpp_2655.cpp)
target_link_libraries(rclcpp_2655 PUBLIC rclcpp::rclcpp ${std_msgs_TARGETS})

add_executable(rclcpp_2664 src/rclcpp_2664/rclcpp_2664.cpp)
target_link_libraries(rclcpp_2664 PUBLIC rclcpp::rclcpp ${std_msgs_TARGETS})

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
  rclcpp_2655
  rclcpp_2664
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
