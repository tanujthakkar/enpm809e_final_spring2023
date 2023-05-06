#! /usr/bin/env python3

# ENPM 809E - RWA4
# Author(s): Tanuj Thakkar (117817539), Aneesh Dandime (118418506)
# Email(s): tanuj@umd.edu, aneeshd@umd.edu


import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.parameter import ParameterType, Parameter

from collections import deque

import PyKDL
from geometry_msgs.msg import Pose

from ariac_msgs.msg import Order as OrderMsg
from ariac_msgs.msg import AdvancedLogicalCameraImage 

from rwa4_group19.rwa4_msgs import Order, KitTrayPose, PartPose

class RWA4Node(Node):
    '''
    Main class for RWA4 ROS2 node

    Args:
        Node (class): ROS2 node class

    Attributes:
        _node_name (str): name of the node
        _orders (deque): queue of orders received
        _tray_poses (dict): dictionary of tray poses
        _part_poses (dict): dictionary of part poses
        _order_sub (Subscriber): subscriber for orders
        _table1_cam_sub (Subscriber): subscriber for table 1 camera
        _table2_cam_sub (Subscriber): subscriber for table 2 camera
        _left_bins_cam_sub (Subscriber): subscriber for left bins camera
        _right_bins_cam_sub (Subscriber): subscriber for right bins camera
        _log_timer (Timer): timer for logging
        _table1_cam_sub_msg (bool): flag to check if table 1 camera message has been received
        _table2_cam_sub_msg (bool): flag to check if table 2 camera message has been received
        _left_bins_cam_sub_msg (bool): flag to check if left bins camera message has been received
        _right_bins_cam_sub_msg (bool): flag to check if right bins camera message has been received
        _log_order (bool): flag to check if order logging is enabled

    Methods:
        __init__(node_name): constructor
        _order_sub_callback(msg): callback function for order subscriber
        _table1_cam_sub_callback(msg): callback function for table 1 camera subscriber
        _table2_cam_sub_callback(msg): callback function for table 2 camera subscriber
        _left_bins_cam_sub_callback(msg): callback function for left bins camera subscriber
        _right_bins_cam_sub_callback(msg): callback function for right bins camera subscriber
        _log_timer_callback(): callback function for log timer
        _multiply_pose(pose1, pose2): multiplies two poses using PyKDL
    '''

    def __init__(self, node_name) -> None:
        '''
        Constructor for RWA4Node class

        Args:
            node_name (str): name of the node

        Returns:
            None
        '''

        super().__init__(node_name)

        self._node_name = node_name
        self._orders = deque()  # queue of orders received
        self._tray_poses = dict()  # dictionary of tray poses
        self._part_poses = dict()  # dictionary of part poses

        self.get_logger().info(f'{self._node_name}: node initialized')

        self.declare_parameter('order_id', None)
        self.order_id = self.get_parameter('order_id').value

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
    
        self._left_bins_cam_sub_msg = False
        self._left_bins_cam_sub = self.create_subscription(AdvancedLogicalCameraImage, 
                                                           '/ariac/sensors/left_bins_camera/image',
                                                           self._left_bins_cam_sub_callback,
                                                           qos_profile)

        self._right_bins_cam_sub_msg = False
        self._right_bins_cam_sub = self.create_subscription(AdvancedLogicalCameraImage,
                                                            '/ariac/sensors/right_bins_camera/image',
                                                            self._right_bins_cam_sub_callback,
                                                            qos_profile)

        self._log_order = False
        self._log_timer = self.create_timer(1.0, self._log_timer_callback)

    def _order_sub_callback(self, msg) -> None:
        '''
        Callback function for order subscriber

        Args:
            msg (OrderMsg): message received from topic /ariac/orders

        Returns:
            None
        '''

        self.get_logger().debug(f'{self._node_name}: received order from topic /ariac/orders')
        order = Order.from_msg(msg)
        self._orders.append(order)
        self.get_logger().debug(f'{self._node_name}: order received:\n{order}')

        self._log_order = True  # log order on next timer callback
        self._table1_cam_sub_msg = False  # reset table1 camera message received flag
        self._table2_cam_sub_msg = False  # reset table2 camera message received flag
        self._left_bins_cam_sub_msg = False  # reset left bins camera message received flag
        self._right_bins_cam_sub_msg = False  # reset right bins camera message received flag

    def _table1_cam_sub_callback(self, msg) -> None:
        '''
        Callback function for table 1 camera subscriber

        Args:
            msg (AdvancedLogicalCameraImage): message received from topic /ariac/sensors/table1_camera/image

        Returns:
            None
        '''

        if len(self._orders) and not self._table1_cam_sub_msg:
            self._table1_cam_sub_msg = True  # only process first message after each order received
            self.get_logger().debug(f'{self._node_name}: received camera image from topic /ariac/sensors/table1_camera/image')

            for tray in msg.tray_poses:
                tray_pose_w = self._multiply_pose(msg.sensor_pose, tray.pose)
                self._tray_poses[tray.id] = KitTrayPose(tray.id, tray_pose_w)
                self.get_logger().debug(f'{self._node_name}: tray pose in world:\n{tray_pose_w}')

    def _table2_cam_sub_callback(self, msg) -> None:
        '''
        Callback function for table 2 camera subscriber

        Args:
            msg (AdvancedLogicalCameraImage): message received from topic /ariac/sensors/table2_camera/image

        Returns:
            None
        '''

        if len(self._orders) and not self._table2_cam_sub_msg:
            self._table2_cam_sub_msg = True  # only process first message after each order received 
            self.get_logger().debug(f'{self._node_name}: received camera image from topic /ariac/sensors/table2_camera/image')

            for tray in msg.tray_poses:
                tray_pose_w = self._multiply_pose(msg.sensor_pose, tray.pose)
                self._tray_poses[tray.id] = KitTrayPose(tray.id, tray_pose_w)
                self.get_logger().debug(f'{self._node_name}: tray pose in world:\n{tray_pose_w}')

    def _left_bins_cam_sub_callback(self, msg) -> None:
        '''
        Callback function for left bins camera subscriber

        Args:
            msg (AdvancedLogicalCameraImage): message received from topic /ariac/sensors/left_bins_camera/image

        Returns:
            None
        '''

        if not self._left_bins_cam_sub_msg:
            self._left_bins_cam_sub_msg = True  # only process first message after each order received 
            self.get_logger().debug(f'{self._node_name}: received camera image from topic /ariac/sensors/left_bins_camera/image')

            for part in msg.part_poses:
                part_pose_w = self._multiply_pose(msg.sensor_pose, part.pose)
                if part.part.type not in self._part_poses:
                    self._part_poses[part.part.type] = dict()
                if part.part.color not in self._part_poses[part.part.type]:
                    self._part_poses[part.part.type][part.part.color] = list()
                self._part_poses[part.part.type][part.part.color].append(PartPose(part.part, part_pose_w))

    def _right_bins_cam_sub_callback(self, msg) -> None:
        '''
        Callback function for right bins camera subscriber

        Args:
            msg (AdvancedLogicalCameraImage): message received from topic /ariac/sensors/right_bins_camera/image

        Returns:
            None
        '''

        if not self._right_bins_cam_sub_msg:
            self._right_bins_cam_sub_msg = True  # only process first message after each order received 
            self.get_logger().debug(f'{self._node_name}: received camera image from topic /ariac/sensors/right_bins_camera/image')

            for part in msg.part_poses:
                part_pose_w = self._multiply_pose(msg.sensor_pose, part.pose)
                if part.part.type not in self._part_poses:
                    self._part_poses[part.part.type] = dict()
                if part.part.color not in self._part_poses[part.part.type]:
                    self._part_poses[part.part.type][part.part.color] = list()
                self._part_poses[part.part.type][part.part.color].append(PartPose(part.part, part_pose_w))
    
    def _log_timer_callback(self) -> None:
        '''
        Callback function for logging timer

        Args:
            None

        Returns:
            None
        '''

        if (len(self._orders) and
            self._log_order and
            self._table1_cam_sub_msg and
            self._table2_cam_sub_msg and
            self._left_bins_cam_sub_msg and
            self._right_bins_cam_sub_msg):
                
                latest_order = self._orders[-1]  # get latest order
                self.get_logger().info(f'\n---------------------- \
                                         \n--- Order {latest_order.id} --- \
                                         \n----------------------')

                tray_id = latest_order.kitting_task.tray_id
                if tray_id in self._tray_poses:
                    self.get_logger().info(f'{self._tray_poses[tray_id]}')
                else:
                    self.get_logger().info(
                        f'{self._node_name}: tray {tray_id} not found')

                parts = latest_order.kitting_task.parts
                for part in parts:
                    if part.part.type in self._part_poses:
                        if part.part.color in self._part_poses[part.part.type]:
                            self.get_logger().info(f'{self._part_poses[part.part.type][part.part.color][0]}')
                            self._part_poses[part.part.type][part.part.color].pop(0)  # remove part from list
                        else:
                            self.get_logger().info(
                                f'{self._node_name}: part {part.part.type} {part.part.color} not found')

                self._log_order = False  # reset flag

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