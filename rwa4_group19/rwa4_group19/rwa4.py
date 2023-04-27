#! /usr/bin/env python3

# ENPM 809E - RWA4
# Author(s): Tanuj Thakkar (117817539), Aneesh Dandime (118418506)
# Email(s): tanuj@umd.edu, aneeshd@umd.edu

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from collections import deque

from ariac_msgs.msg import Order as OrderMsg
from rwa4_group19.rwa4_msgs import Order

class RWA4Node(Node):
    '''
    Main class for RWA4 ROS2 node

    Args:
        Node (class): ROS2 node class
    '''

    def __init__(self, node_name):
        super().__init__(node_name)

        self._node_name = node_name
        self._orders = deque()  # queue of orders received

        self.get_logger().info(f'{self._node_name}: node initialized')

        # Subscribers
        self._order_sub = self.create_subscription(OrderMsg, '/ariac/orders',
                                                   self._order_sub_callback,
                                                   10)

    def _order_sub_callback(self, message):
        order = Order.from_msg(message)
        self._orders.append(order)
        self.get_logger().info(f'{self._node_name}: received order from topic /ariac/orders:\n{order}')

def main(args=None):
    '''
    Main function for RWA4

    Args:
        args (Any, optional): ROS2 arguments. Defaults to None.
    '''

    rclpy.init(args=args)  # initialize ROS2

    node = RWA4Node('rwa4')  # create node

    rclpy.spin(node)  # spin node
    node.destroy_node()  # destroy node

    rclpy.shutdown()  # shutdown ROS2

if __name__ == '__main__':
    main()