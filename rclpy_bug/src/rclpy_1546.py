#! /usr/bin/env python3
import rclpy
import rclpy.parameter


def main(args=None):
    rclpy.init(args=args)

    output = rclpy.parameter.parameter_dict_from_yaml_file(parameter_file='/root/ros2_src_jazzy/test.yaml', use_wildcard=True)
    print(output)
    rclpy.shutdown()
