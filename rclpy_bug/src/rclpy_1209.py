from typing import Optional
import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import LifecyclePublisher
from rclpy.lifecycle import LifecycleState
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.timer import Timer

from std_msgs.msg import Bool

class LifecycleSubDemo(LifecycleNode):
  
  def __init__(self, node_name, **kwargs):
    super().__init__(node_name, **kwargs)

    self._count: int = 0
    self._pub: Optional[LifecyclePublisher] = None
    self._timer: Optional[Timer] = None

  def publish(self):
    self._pub.publish(Bool(data=False))
  

  def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
    self._pub = self.create_lifecycle_publisher(Bool, "/test", 1)
    self._timer = self.create_timer(1.0, self.publish, autostart=False)
    return TransitionCallbackReturn.SUCCESS

  def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
    # publisher automatically activated by parent class transition method
    r = super().on_activate(state)
    self._timer.reset()
    return r
  
  def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
    self._timer.cancel()
    return super().on_deactivate(state)

  def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
    self.destroy_timer(self._timer)
    self.destroy_publisher(self._pub)
    return TransitionCallbackReturn.SUCCESS
  
  def on_error(self, state: LifecycleState) -> TransitionCallbackReturn:
    self.get_logger().error("on_error triggered")
    return TransitionCallbackReturn.SUCCESS


def main():
    rclpy.init()
    lc_demo_node = LifecycleSubDemo('lc_demo')
    rclpy.spin(lc_demo_node)

    #while rclpy.ok():
    #  lc_demo_node = LifecycleSubDemo('lc_demo')
    #  try:
    #    rclpy.spin(lc_demo_node)
    #  except rclpy._rclpy_pybind11.RCLError:
    #    pass

    rclpy.shutdown()

if __name__ == '__main__':
  main()
