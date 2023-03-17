import rclpy.node
import rclpy

def main(args=None):
    rclpy.init()
    node = rclpy.node.Node("mynode", allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
    rclpy.spin(node)

if __name__ == '__main__':
    main()