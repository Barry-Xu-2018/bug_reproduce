import rclpy


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('minimal_client')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
