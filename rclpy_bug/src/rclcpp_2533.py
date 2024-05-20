import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time

class ClockPublisher(Node):
    def __init__(self):
        super().__init__('clock_publisher')
        self.publisher_ = self.create_publisher(Clock, '/clock', 10)
        
        # Use a timer to publish the clock message at a regular interval
        self.timer_period = 3  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.sim_time = 0.0  # starting simulated time in seconds

    def timer_callback(self):
        clock_msg = Clock()
        clock_msg.clock = Time(sec=int(self.sim_time), nanosec=int((self.sim_time - int(self.sim_time)) * 1e9))
        self.publisher_.publish(clock_msg)
        
        # Increment simulated time
        self.sim_time += self.timer_period

def main(args=None):
    rclpy.init(args=args)
    node = ClockPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
