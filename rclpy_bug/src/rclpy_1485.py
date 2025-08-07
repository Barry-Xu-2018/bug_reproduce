#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from std_msgs.msg import Empty
import threading
from rclpy.qos import qos_profile_sensor_data

timer = False


class MyNode(Node):
    def __init__(self):
        super().__init__("my_node")

        self._timer_callback_group = MutuallyExclusiveCallbackGroup()
        self._default_callback_group = ReentrantCallbackGroup()
        self._heartbeat_timeout = 0.5678
        self._timer_count = 0

        if timer == False:
            self._sub_vehicle_status = self.create_subscription(
                Empty,
                "/test",
                self.callback_status,
                qos_profile_sensor_data,
            )
        else:
            self._timer = self.create_timer(0.5, self.callback_timer)

        self.status_count = 0

    def callback_status(self, msg):
        self.status_count += 1
        self.get_logger().info(f"Callback status {threading.get_ident()}")
        if self.status_count == 10:
            self.enter_enabled()
            self.status_count = 0

    def callback_timer(self):
        self.get_logger().info(f"Callback timer {threading.get_ident()}")
        self.enter_enabled()

    def enter_enabled(self):
        self.get_logger().info(f"Entering enabled {threading.get_ident()}")
        self._heartbeat_watchdog = self.create_timer(
            self._heartbeat_timeout,
            self._heartbeat_watchdog_callback,
        )
        self._heartbeat_watchdog2 = self.create_timer(
            self._heartbeat_timeout,
            self._heartbeat_watchdog_callback2,
        )

    def exit_enabled(self):
        self.get_logger().info(f"Exiting enabled {threading.get_ident()}")
        self.get_logger().info(f"Cancelling timer 1")
        self._heartbeat_watchdog.cancel()
        self.get_logger().info(f"Cancelling timer 2")
        self._heartbeat_watchdog2.cancel()
        self.get_logger().info(f"Cancelling timers done")

    def _heartbeat_watchdog_callback(self):
        self.get_logger().info("CB1!")
        self.exit_enabled()

    def _heartbeat_watchdog_callback2(self):
        self.get_logger().info("CB2!")
        self.exit_enabled()


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor(1)
    node = MyNode()

    try:
        rclpy.spin(node, executor)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()

