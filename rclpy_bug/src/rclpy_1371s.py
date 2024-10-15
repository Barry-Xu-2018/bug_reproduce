import time

import rclpy
from rclpy.action import ActionServer
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

#from custom_action_interfaces.action import Fibonacci
from example_interfaces.action import Fibonacci

class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1])
            self.get_logger().info('Feedback: {0}'.format(feedback_msg.sequence))
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.1)

        goal_handle.succeed()

        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        return result


def main(args=None):
    try:
        with rclpy.init(args=args):
            fibonacci_action_server = FibonacciActionServer()

            rclpy.spin(fibonacci_action_server)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
