import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

#from custom_action_interfaces.action import Fibonacci
from example_interfaces.action import Fibonacci
import time

class FibonacciActionClient(Node):

    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

    def feedback(self, msg):
        print(msg)

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()

        return self._action_client.send_goal_async(goal_msg,self.feedback)


def main(args=None):
    try:
        with rclpy.init(args=args):
            action_client = FibonacciActionClient()
            while rclpy.ok:

                future = action_client.send_goal(1)
                # suspicious function that make the client node memory grow
                rclpy.spin_until_future_complete(action_client, future)
                time.sleep(0.05)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
