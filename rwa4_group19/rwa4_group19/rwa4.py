#! /usr/bin/env python3

# ENPM 809E - RWA4
# Author(s): Tanuj Thakkar (117817539), Aneesh Dandime (118418506)
# Email(s): tanuj@umd.edu, aneeshd@umd.edu


import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from collections import deque

import PyKDL
from geometry_msgs.msg import Pose

from ariac_msgs.msg import Order as OrderMsg
from ariac_msgs.msg import AdvancedLogicalCameraImage 

from rwa4_group19.rwa4_msgs import Order, KitTrayPose

class RWA4Node(Node):
    '''
    Main class for RWA4 ROS2 node

    Args:
        Node (class): ROS2 node class
    '''

    def __init__(self, node_name) -> None:
        super().__init__(node_name)

        self._node_name = node_name
        self._orders = deque()  # queue of orders received
        self._tray_poses = dict()  # dictionary of tray poses

        self.get_logger().info(f'{self._node_name}: node initialized')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=10)

        # Subscribers
        self._order_sub = self.create_subscription(OrderMsg, '/ariac/orders',
                                                   self._order_sub_callback,
                                                   10)

        self._table1_cam_sub_msg = False
        self._table1_cam_sub = self.create_subscription(AdvancedLogicalCameraImage, 
                                                        '/ariac/sensors/table1_camera/image',
                                                        self._table1_cam_sub_callback,
                                                        qos_profile)

        self._table2_cam_sub_msg = False
        self._table2_cam_sub = self.create_subscription(AdvancedLogicalCameraImage, 
                                                        '/ariac/sensors/table2_camera/image',
                                                        self._table2_cam_sub_callback,
                                                        qos_profile)

    def _order_sub_callback(self, message) -> None:
        order = Order.from_msg(message)
        self._orders.append(order)
        self.get_logger().info(f'{self._node_name}: received order from topic /ariac/orders:\n{order}')

    def _table1_cam_sub_callback(self, msg) -> None:
        if not self._table1_cam_sub_msg:
            self._table1_cam_sub_msg = True  # only process first message
            self.get_logger().info(f'{self._node_name}: received camera image from topic /ariac/sensors/table1_camera/image')
            
            # Get current order tray id
            tray_id = self._orders[0].kitting_task.tray_id if len(self._orders) > 0 else None

            # Find tray in camera image msg
            for tray_pose in msg.tray_poses:
                if tray_id and tray_pose.id == tray_id:
                    tray_pose_w = self._multiply_pose(msg.sensor_pose, tray_pose)
                    self._tray_poses[tray_id] = KitTrayPose(tray_id, tray_pose_w)
                    self.get_logger().info(f'{self._node_name}: tray pose in world:\n{tray_pose_w}')
                    break

    def _table2_cam_sub_callback(self, msg) -> None:
        if not self._table2_cam_sub_msg:
            self._table2_cam_sub_msg = True  # only process first message
            self.get_logger().info(f'{self._node_name}: received camera image from topic /ariac/sensors/table2_camera/image')

            # Get current order tray id
            tray_id = self._orders[0].kitting_task.tray_id if len(self._orders) > 0 else None

            # Find tray in camera image msg
            for tray_pose in msg.tray_poses:
                if tray_id and tray_pose.id == tray_id:
                    tray_pose_w = self._multiply_pose(msg.sensor_pose, tray_pose)
                    self._tray_poses[tray_id] = KitTrayPose(tray_id, tray_pose_w)
                    self.get_logger().info(f'{self._node_name}: tray pose in world:\n{tray_pose_w}')
                    break

    def _multiply_pose(self, pose1: Pose, pose2: Pose) -> Pose:
        '''
        Use KDL to multiply two poses together.
        Args:
            pose1 (Pose): Pose of the first frame
            pose2 (Pose): Pose of the second frame
        Returns:
            Pose: Pose of the resulting frame
        '''

        orientation1 = pose1.orientation
        frame1 = PyKDL.Frame(
            PyKDL.Rotation.Quaternion(
                orientation1.x, orientation1.y, orientation1.z, orientation1.w),
            PyKDL.Vector(pose1.position.x, pose1.position.y, pose1.position.z))

        orientation2 = pose2.orientation
        frame2 = PyKDL.Frame(
            PyKDL.Rotation.Quaternion(
                orientation2.x, orientation2.y, orientation2.z, orientation2.w),
            PyKDL.Vector(pose2.position.x, pose2.position.y, pose2.position.z))

        frame3 = frame1 * frame2

        # return the resulting pose from frame3
        pose = Pose()
        pose.position.x = frame3.p.x()
        pose.position.y = frame3.p.y()
        pose.position.z = frame3.p.z()

        q = frame3.M.GetQuaternion()
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]

        return pose

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