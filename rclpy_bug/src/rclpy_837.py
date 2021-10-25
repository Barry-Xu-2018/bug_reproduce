import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class DynamicPublisher(Node):

    def __init__(self):
        super().__init__("dynamic_publisher")
        self.publisher_ = self.create_publisher(String, "test_topic", 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
#        self.timer2 = self.create_timer(timer_period, self.timer_callback2)
#        self.n = 0

    def timer_callback(self):
        msg = String()
        msg.data = "1:Publishing message {}".format(self.i)
        self.publisher_.publish(msg)
        self.get_logger().info('1:Publishing message {}'.format(self.i))
        self.i += 1

        if self.i == 5:
            #for event_callback in self.publisher_.event_handlers:
            #    self.remove_waitable(event_callback)
            self.destroy_publisher(self.publisher_)
            self.timer.cancel()
            #self.destroy_publisher(self.publisher_)
#    def timer_callback2(self):
#        msg = String()
#        msg.data = "1:Publishing message {}".format(self.n)
#        self.publisher_.publish(msg)
#        self.get_logger().info('1:Publishing message {}'.format(self.n))
#        self.n += 1
#
#        if self.n == 4:
#            self.timer2.cancel()
#            #self.destroy_publisher(self.publisher_)


def main(args=None):
    rclpy.init(args=args)

    dynamic_publisher = DynamicPublisher()

    rclpy.spin(dynamic_publisher)

    dynamic_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()