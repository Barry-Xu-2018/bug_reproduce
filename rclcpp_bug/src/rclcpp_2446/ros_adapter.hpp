// rosAdapter.hpp
#ifndef ROS_ADAPTER_HPP
#define ROS_ADAPTER_HPP

#include <string>
#include <iostream>
#include "rclcpp/type_adapter.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"
#include "std_msgs/msg/header.hpp"
#include "bug_reproduce_interfaces/msg/test_msg.hpp"

#define ROS_TYPE
#ifdef ROS_TYPE
// fail
using MsgT = bug_reproduce_interfaces::msg::TestMsg;
#else
// work fine
using MsgT = std::string;
#endif // ROS_TYPE

template <>
struct rclcpp::TypeAdapter<MsgT, std_msgs::msg::ByteMultiArray>
{
    using is_specialized = std::true_type;
    using custom_type = MsgT;
    using ros_message_type = std_msgs::msg::ByteMultiArray;

    static void
    convert_to_ros_message(
        const custom_type &source,
        ros_message_type &destination)
    {
        std::cout << "convert_to_ros_message!!!" << std::endl;
#ifdef ROS_TYPE
        destination.data.resize(12);
        destination.data[0] = source.id;
        destination.data[1] = source.time;
#else
        destination.data.resize(source.size());
        memcpy(destination.data.data(), source.data(), source.size());
#endif
    }

    static void
    convert_to_custom(
        const ros_message_type &source,
        custom_type &destination)
    {
        std::cout << "convert_to_custom!!!" << std::endl;
#ifdef ROS_TYPE
        std::cout << "source size = " << source.data.size() << std::endl;
        destination.id = source.data[0];
        std::cout << "destination.id = " << destination.id << std::endl;
        destination.time = source.data[4];
#else
        destination.resize(source.data.size());
        memcpy(destination.data(), source.data.data(), source.data.size());
#endif
    }
};

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(MsgT, std_msgs::msg::ByteMultiArray);

#endif // ROS_ADAPTER_HPP
