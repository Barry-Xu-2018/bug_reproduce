cmake_minimum_required(VERSION 3.8)
project(bug_reproduce_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

#Glob together all the message files
set(MSG_DIR "${CMAKE_CURRENT_SOURCE_DIR}/msg")
file(GLOB MSG_FILES RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "${MSG_DIR}/*.msg")

# Glob together all the service files
set(SRV_DIR "${CMAKE_CURRENT_SOURCE_DIR}/srv")
file(GLOB SRV_FILES RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "${SRV_DIR}/*.srv")

# Glob together all the action files
set(ACT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/action")
file(GLOB ACT_FILES RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "${ACT_DIR}/*.action")

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  ${SRV_FILES}
  ${ACT_FILES})

ament_package()
