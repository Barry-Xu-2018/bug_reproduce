#! /usr/bin/env python3
import rclpy
import rclpy.parameter


def main(args=None):
    rclpy.init(args=args)

    output = rclpy.parameter.parameter_dict_from_yaml_file(
        parameter_file='/tmp/test.yaml', use_wildcard=True, target_nodes=['param_test_target3'])
    import pprint
    pprint.pprint(output, indent=2)
    rclpy.shutdown()
