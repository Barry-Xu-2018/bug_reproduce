import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.executors import ExternalShutdownException

class DummyPublisher(Node):
    def __init__(self):
        super().__init__('dummy_publisher')
        self.publisher_ = self.create_publisher(String, 'dummy_topic', 10)
        self.timer = self.create_timer(1, self.publish_dummy_string)
        self.get_logger().info('Dummy Publisher Node has been started.')  # want this to be logged in file

    def publish_dummy_string(self):
        msg = String()
        msg.data = 'Hello, this is a dummy string!'
        self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)  # the above only gets logged if this is uncommented

def main(args=None):
    rclpy.init(args=args)
    node = DummyPublisher()
    try:
      rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
      node.destroy_node()
      rclpy.try_shutdown()

if __name__ == '__main__':
    main()
