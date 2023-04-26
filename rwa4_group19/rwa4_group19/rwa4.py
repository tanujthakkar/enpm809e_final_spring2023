#! /usr/bin/env python3

import rclpy
from rclpy.node import Node


class RWA4Node(Node):
    '''
    Main class for RWA4 ROS2 node

    Args:
        Node (class): ROS2 node class
    '''

    def __init__(self, node_name):
        super().__init__(node_name)
        self.get_logger().info(f'{node_name}: node initialized')


def main(args=None):
    '''
    Main function for RWA4

    Args:
        args (Any, optional): ROS2 arguments. Defaults to None.
    '''

    rclpy.init(args=args)
    node = RWA4Node('rwa4')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()