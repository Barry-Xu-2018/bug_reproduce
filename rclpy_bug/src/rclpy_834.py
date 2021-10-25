# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from example_interfaces.srv import AddTwoInts

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

import time
import threading

class AddTwoIntsServer(Node):

    def __init__(self):
        super().__init__('add_two_ints_server')
        #from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        #self.srv2 = self.create_service(AddTwoInts, 'add_two_ints2', self.add_two_ints_callback2)


    async def add_two_ints_callback(self, request, response):
        self.get_logger().info('1:thread id %d' % threading.current_thread().ident)
        response.sum = request.a + request.b
        self.get_logger().info('1:Incoming request\na: %d b: %d' % (request.a, request.b))

        time.sleep(6)
        self.get_logger().info('1:sending result')
        return response

class AddTwoIntsServer2(Node):

    def __init__(self):
        super().__init__('add_two_ints_server2')
        #from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
        self.srv = self.create_service(AddTwoInts, 'add_two_ints2', self.add_two_ints_callback2)

    def add_two_ints_callback2(self, request, response):
        self.get_logger().info('2:thread id %d' % threading.current_thread().ident)
        response.sum = request.a + request.b
        self.get_logger().info('2:Incoming request\na: %d b: %d' % (request.a, request.b))

        time.sleep(2)
        self.get_logger().info('2:sending result')
        return response


def main(args=None):
    rclpy.init(args=args)

    node = AddTwoIntsServer()
    node2 = AddTwoIntsServer2()

    exec = MultiThreadedExecutor()
    exec.add_node(node2)

    try:
        rclpy.spin(node, exec)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    # (optional - Done automatically when node is garbage collected)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()