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
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor

from collections import deque
import time

import PyKDL
from geometry_msgs.msg import Pose
from std_srvs.srv import Trigger

from ariac_msgs.msg import Order as OrderMsg
from ariac_msgs.msg import AdvancedLogicalCameraImage, CompetitionState
from ariac_msgs.srv import ChangeGripper, VacuumGripperControl
from competitor_interfaces.msg import Robots as RobotsMsg
from competitor_interfaces.srv import EnterToolChanger, ExitToolChanger, PickupTray, \
                                      MoveTrayToAGV, PlaceTrayOnAGV, RetractFromAGV, \
                                      PickupPart, MovePartToAGV, PlacePartInTray

from rwa4_group19.rwa4_msgs import Order, KitTrayPose, PartPose
from robot_commander.robot_commander_interface import RobotCommanderInterface

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
        self._part_poses = {'left_bins' : dict(),
                            'right_bins' : dict()}  # dictionary of part poses
        self._tables = {'kts1' : [],
                        'kts2' : []}  # dictionary of tables for trays

        self.get_logger().info(f'{self._node_name}: node initialized')

        self.declare_parameter('order_id', None)
        self._order_id = int(self.get_parameter('order_id').value)
        self.get_logger().info(f'{self._node_name}: order_id: {self._order_id}')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=10)
        sub_group = MutuallyExclusiveCallbackGroup()  # subscriber callback group

        # Subscribers
        self._order_sub = self.create_subscription(OrderMsg, '/ariac/orders',
                                                   self._order_sub_callback,
                                                   10, callback_group=sub_group)

        self._table1_cam_sub_msg = False
        self._table1_cam_sub = self.create_subscription(AdvancedLogicalCameraImage, 
                                                        '/ariac/sensors/table1_camera/image',
                                                        self._table1_cam_sub_callback,
                                                        qos_profile, callback_group=sub_group)

        self._table2_cam_sub_msg = False
        self._table2_cam_sub = self.create_subscription(AdvancedLogicalCameraImage, 
                                                        '/ariac/sensors/table2_camera/image',
                                                        self._table2_cam_sub_callback,
                                                        qos_profile, callback_group=sub_group)
    
        self._left_bins_cam_sub_msg = False
        self._left_bins_cam_sub = self.create_subscription(AdvancedLogicalCameraImage, 
                                                           '/ariac/sensors/left_bins_camera/image',
                                                           self._left_bins_cam_sub_callback,
                                                           qos_profile, callback_group=sub_group)

        self._right_bins_cam_sub_msg = False
        self._right_bins_cam_sub = self.create_subscription(AdvancedLogicalCameraImage,
                                                            '/ariac/sensors/right_bins_camera/image',
                                                            self._right_bins_cam_sub_callback,
                                                            qos_profile, callback_group=sub_group)

        self._log_order = False
        self._log_timer = self.create_timer(0.1, self._log_timer_callback)

        timer_group = MutuallyExclusiveCallbackGroup()  # timer callback group
        service_group = MutuallyExclusiveCallbackGroup()  # service callback group

        # Flag to indicate if the kit has been completed
        self._kit_completed = False
        self._competition_started = False
        self._competition_state = None

        self.create_subscription(CompetitionState, '/ariac/competition_state',
                                 self._competition_state_cb, 1)
    
        self._robot_action_timer = self.create_timer(1, self._robot_action_timer_callback,
                                                     callback_group=timer_group)
        
        self._start_competition_client = self.create_client(
            Trigger, '/ariac/start_competition')
    
        # Service client for moving the floor robot to the home position
        self._floor_robot_go_home_client = self.create_client(
            Trigger, '/competitor/floor_robot/go_home',
            callback_group=service_group)

        # Service client for entering the gripper slot
        self._enter_tool_changer_client = self.create_client(
            EnterToolChanger, '/competitor/floor_robot/enter_tool_changer',
            callback_group=service_group)

        self._exit_tool_changer_client = self.create_client(
            ExitToolChanger, '/competitor/floor_robot/exit_tool_changer',
            callback_group=service_group)

        self._pickup_tray_client = self.create_client(
            PickupTray, '/competitor/floor_robot/pickup_tray',
            callback_group=service_group)

        self._move_tray_to_agv_client = self.create_client(
            MoveTrayToAGV, '/competitor/floor_robot/move_tray_to_agv',
            callback_group=service_group)

        self._place_tray_on_agv_client = self.create_client(
            PlaceTrayOnAGV, '/competitor/floor_robot/place_tray_on_agv',
            callback_group=service_group)

        self._retract_from_agv_client = self.create_client(
            RetractFromAGV, '/competitor/floor_robot/retract_from_agv',
            callback_group=service_group)

        self._pickup_part_client = self.create_client(
            PickupPart, '/competitor/floor_robot/pickup_part',
            callback_group=service_group)

        self._move_part_to_agv_client = self.create_client(
            MovePartToAGV, '/competitor/floor_robot/move_part_to_agv',
            callback_group=service_group)

        self._place_part_in_tray_client = self.create_client(
            PlacePartInTray, '/competitor/floor_robot/place_part_in_tray',
            callback_group=service_group)

        # self._agv3_lock_tray_client = self.create_client(
        #     Trigger, '/ariac/agv3_lock_tray',
        #     callback_group=service_group)

        # self._move_agv3_client = self.create_client(
        #     MoveAGV, '/ariac/move_agv3',
        #     callback_group=service_group)

        self._floor_robot_change_gripper_client = self.create_client(
            ChangeGripper, '/ariac/floor_robot_change_gripper',
            callback_group=service_group)

        self._floor_robot_enable_gripper_client = self.create_client(
            VacuumGripperControl, '/ariac/floor_robot_enable_gripper',
            callback_group=service_group)
    
        while self._competition_state != CompetitionState.READY:
            self.get_logger().info('Waiting for competition to be ready...')
            rclpy.spin_once(self)

        if self._competition_state == CompetitionState.READY:
            self.get_logger().info('Competition is ready, starting...')
            self.start_competition()
            while self._competition_state != CompetitionState.ORDER_ANNOUNCEMENTS_DONE:
                self.get_logger().info('Waiting for competition to start...')
                rclpy.spin_once(self)

    def _robot_action_timer_callback(self):
        '''
        Callback for the timer that triggers the robot actions
        '''
        
        if self._competition_state != CompetitionState.ORDER_ANNOUNCEMENTS_DONE:
            return

        # exit the callback if the kit is completed
        if self._kit_completed:
            return
        
        # Get order details
        if self._competition_started:
            self.get_logger().info(f'{self._orders}')
            curr_order = None
            for order in self._orders:
                self.get_logger().info(f'{self._order_id} {order.id}')
                if int(order.id) == int(self._order_id):
                    curr_order = order
                    break
            self.get_logger().info(f'{self._node_name}: curr_order: {curr_order}')
        
        agv = "agv" + str(curr_order.kitting_task.agv_number)
        self.get_logger().info(f'{self._node_name}: agv: {agv}')

        tray_id = curr_order.kitting_task.tray_id
        if tray_id in self._tray_poses:
            self.get_logger().info(f'{self._tray_poses[tray_id]}')
        else:
            self.get_logger().info(
                f'{self._node_name}: tray {tray_id} not found')

        tray_pose = self._tray_poses[tray_id].pose

        tray_table = None
        for table in self._tables:
            if tray_id in self._tables[table]:
                tray_table = table
                break

        # move robot home
        self.move_robot_home("floor_robot")

        # change gripper tool to tray gripper
        self.goto_tool_changer("floor_robot", tray_table, "trays")
        self.retract_from_tool_changer("floor_robot", tray_table, "trays")

        # move to tray and transport to agv
        self.pickup_tray("floor_robot", tray_id, tray_pose, tray_table)
        self.move_tray_to_agv("floor_robot", tray_pose, agv)
        self.place_tray("floor_robot", agv, tray_id)
        self.retract_from_agv("floor_robot", agv)

        # change gripper tool to part gripper
        self.goto_tool_changer("floor_robot", tray_table, "parts")
        self.retract_from_tool_changer("floor_robot", tray_table, "parts")

        # service parts
        for part in curr_order.kitting_task.parts:
            for bin in self._part_poses:
                if part.part.type in self._part_poses[bin]:
                    if part.part.color in self._part_poses[bin][part.part.type]:
                        curr_part = self._part_poses[bin][part.part.type][part.part.color][0]  # get first part in list
                        quadrant = part.quadrant

                        self.pickup_part("floor_robot", part.part.type, part.part.color, curr_part.pose, bin)
                        self.move_part_to_agv("floor_robot", curr_part.pose, agv, quadrant)
                        self.place_part_in_tray("floor_robot", agv, quadrant)
                        self.retract_from_agv("floor_robot", agv)

                        self.get_logger().info(f'{self._node_name}: part {part.part.type} {part.part.color} found in bin {bin}')
                        self._part_poses[bin][part.part.type][part.part.color].pop(0)  # remove part from list
                    else:
                        self.get_logger().info(f'{self._node_name}: part {part.part.type} {part.part.color} not found')

        self.get_logger().info(f'{self._node_name}: kitting completed')

        # to ignore function calls in this callback
        self._kit_completed = True
    
    def _competition_state_cb(self, msg: CompetitionState):
        '''
        /ariac/competition_state topic callback function

        Args:
            msg (CompetitionState): CompetitionState message
        '''
        self._competition_state = msg.competition_state

    def start_competition(self):
        '''
        Start the competition
        '''
        self.get_logger().info('Waiting for competition state READY')

        request = Trigger.Request()
        future = self._start_competition_client.call_async(request)

        # Wait until the service call is completed
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            self.get_logger().info('Started competition.')
            self._competition_started = True
        else:
            self.get_logger().warn('Unable to start competition')

    def move_robot_home(self, robot_name):
        '''Move one of the robots to its home position.

        Arguments:
            robot_name -- Name of the robot to move home
        '''
        request = Trigger.Request()

        if robot_name == 'floor_robot':
            if not self._floor_robot_go_home_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().error('Robot commander node not running')
                return

            future = self._floor_robot_go_home_client.call_async(request)
        else:
            self.get_logger().error(f'Robot name: ({robot_name}) is not valid')
            return

        # Wait until the service call is completed
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            self.get_logger().info(f'Moved {robot_name} to home position')
        else:
            self.get_logger().warn(future.result().message)
    
    def goto_tool_changer(self, robot, station, gripper_type):
        '''
        Move the end effector inside the gripper slot.

        Args:
            robot (str): Robot name
            station (str): Gripper station name
            gripper_type (str): Gripper type

        Raises:
            KeyboardInterrupt: Exception raised when the user presses Ctrl+C
        '''

        self.get_logger().info('Move inside gripper slot service called')

        request = EnterToolChanger.Request()

        if robot == "floor_robot":
            request.robot = RobotsMsg.FLOOR_ROBOT
        else:
            raise ValueError('Invalid robot name')

        request.station = station
        request.gripper_type = gripper_type

        future = self._enter_tool_changer_client.call_async(request)

        try:
            rclpy.spin_until_future_complete(self, future)
        except KeyboardInterrupt as kb_error:
            raise KeyboardInterrupt from kb_error

        if future.result() is not None:
            response = future.result()
            if response:
                self.get_logger().info('Robot is inside tool changer')
                if gripper_type == "trays":
                    self.get_logger().info('Changing gripper tool to TRAY_GRIPPER')
                    self.floor_robot_change_gripper("TRAY_GRIPPER")
                else:
                    self.get_logger().info('Changing gripper tool to PART_GRIPPER')
                    self.floor_robot_change_gripper("PART_GRIPPER")
        else:
            self.get_logger().error(
                f'Service call failed {future.exception()}')
            self.get_logger().error('Unable to move the robot to the tool changer')
        
    def retract_from_tool_changer(self, robot, station, gripper_type):
        '''
        Move the end effector outside the gripper slot.

        Args:
            station (str): Gripper station name
            gripper_type (str): Gripper type

        Raises:
            KeyboardInterrupt: Exception raised when the user presses Ctrl+C
        '''

        self.get_logger().info('Move outside gripper slot service called')

        request = ExitToolChanger.Request()

        if robot == "floor_robot":
            request.robot = RobotsMsg.FLOOR_ROBOT
        else:
            raise ValueError('Invalid robot name')

        request.station = station
        request.gripper_type = gripper_type

        future = self._exit_tool_changer_client.call_async(request)

        try:
            rclpy.spin_until_future_complete(self, future)
        except KeyboardInterrupt as kb_error:
            raise KeyboardInterrupt from kb_error

        if future.result() is not None:
            response = future.result()
            if response:
                self.get_logger().info('Robot is outside tool changer')
        else:
            self.get_logger().error(
                f'Service call failed {future.exception()}')
            self.get_logger().error('Unable to move the robot to the tool changer')
    
    def pickup_tray(self, robot, tray_id, tray_pose, station):
        '''
        Pickup tray from table

        Args:
            robot (str): Robot name
            tray_id (int): Tray ID
            tray_pose (Pose): Pose of the tray
            station (str): Table name

        Returns:
            bool: True if successful, False otherwise
        '''

        request = PickupTray.Request()
        if robot == "floor_robot":
            request.robot = RobotsMsg.FLOOR_ROBOT
        else:
            raise ValueError('Invalid robot name')
        request.tray_id = tray_id
        request.tray_pose = tray_pose
        request.tray_station = station

        if robot == "floor_robot":
            request.robot = RobotsMsg.FLOOR_ROBOT
        else:
            raise ValueError('Invalid robot name')

        future = self._pickup_tray_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            self.get_logger().info(f'Enabling gripper for floor robot')
            self.floor_robot_gripper_control(True)
            self.get_logger().info(f'Picked up tray {tray_id} from table {station}')
            return True
        else:
            self.get_logger().error(f'Unable to pick up tray {tray_id} from table {station}')
            return False
    
    def move_tray_to_agv(self, robot, tray_pose, agv):
        '''
        Move tray to AGV

        Args:
            robot (str): Robot name
            tray_pose (Pose): Pose of the tray
            agv (str): AGV name

        Returns:
            bool: True if successful, False otherwise
        '''

        request = MoveTrayToAGV.Request()
        if robot == "floor_robot":
            request.robot = RobotsMsg.FLOOR_ROBOT
        else:
            raise ValueError('Invalid robot name')
        request.tray_pose = tray_pose
        request.agv = agv

        future = self._move_tray_to_agv_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            self.get_logger().info(f'Moved tray to {agv}')
            return True
        else:
            self.get_logger().error(f'Unable to move tray to {agv}')
            return False
    
    def place_tray(self, robot, agv, tray_id):
        '''
        Place tray on AGV

        Args:
            robot (str): Robot name
            agv (str): AGV name
            tray_id (int): Tray ID

        Returns:
            bool: True if successful, False otherwise
        '''

        request = PlaceTrayOnAGV.Request()
        if robot == "floor_robot":
            request.robot = RobotsMsg.FLOOR_ROBOT
        else:
            raise ValueError('Invalid robot name')
        request.agv = agv
        request.tray_id = tray_id

        future = self._place_tray_on_agv_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            self.get_logger().info(f'Disabling gripper for floor robot')
            self.floor_robot_gripper_control(False)
            self.get_logger().info(f'Placed tray {tray_id} on {agv}')
            return True
        else:
            self.get_logger().error(f'Unable to place tray {tray_id} on {agv}')
            return False
    
    def retract_from_agv(self, robot, agv):
        '''
        Retract from AGV

        Args:
            robot (str): Robot name
            agv (str): AGV name

        Returns:
            bool: True if successful, False otherwise
        '''

        request = RetractFromAGV.Request()
        if robot == "floor_robot":
            request.robot = RobotsMsg.FLOOR_ROBOT
        else:
            raise ValueError('Invalid robot name')
        request.agv = agv

        future = self._retract_from_agv_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            self.get_logger().info(f'Retracted from {agv}')
            return True
        else:
            self.get_logger().error(f'Unable to retract from {agv}')
            return False
    
    def pickup_part(self, robot, part_type, part_color, part_pose, bin_side):
        '''
        Pickup part from bin

        Args:
            robot (str): Robot name
            part_type (str): Part type
            part_color (str): Part color
            part_pose (Pose): Pose of the part
            bin_side (str): Bin side

        Returns:
            bool: True if successful, False otherwise
        '''

        request = PickupPart.Request()
        if robot == "floor_robot":
            request.robot = RobotsMsg.FLOOR_ROBOT
        else:
            raise ValueError('Invalid robot name')
        request.part_type = part_type
        request.part_color = part_color
        request.part_pose = part_pose
        request.bin_side = bin_side

        future = self._pickup_part_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            self.get_logger().info(f'Enabling gripper for floor robot')
            self.floor_robot_gripper_control(True)
            self.get_logger().info(f'Picked up {part_color} {part_type} from {bin_side}')
            return True
        else:
            self.get_logger().error(
                f'Unable to pick up {part_color} {part_type} from {bin_side}')
            return False
    
    def move_part_to_agv(self, robot, part_pose, agv, quadrant):
        '''
        Move part to AGV

        Args:
            robot (str): Robot name
            part_pose (Pose): Pose of the part
            agv (str): AGV name
            quadrant (int): Quadrant of the AGV

        Returns:
            bool: True if successful, False otherwise
        '''

        request = MovePartToAGV.Request()
        if robot == "floor_robot":
            request.robot = RobotsMsg.FLOOR_ROBOT
        else:
            raise ValueError('Invalid robot name')
        request.part_pose = part_pose
        request.agv = agv
        request.quadrant = quadrant

        future = self._move_part_to_agv_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            self.get_logger().info(f'Moved part to {agv}')
            return True
        else:
            self.get_logger().error(f'Unable to move part to {agv}')
            return False

    def place_part_in_tray(self, robot, agv, quadrant):
        '''
        Place part in tray

        Args:
            robot (str): Robot name
            agv (str): AGV name
            quadrant (int): Quadrant of the AGV

        Returns:
            bool: True if successful, False otherwise
        '''

        request = PlacePartInTray.Request()
        if robot == "floor_robot":
            request.robot = RobotsMsg.FLOOR_ROBOT
        else:
            raise ValueError('Invalid robot name')
        request.agv = agv
        request.quadrant = quadrant

        future = self._place_part_in_tray_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            self.get_logger().info(f'Disabling gripper for floor robot')
            self.floor_robot_gripper_control(False)
            self.get_logger().info(f'Placed part in {agv}')
            return True
        else:
            self.get_logger().error(f'Unable to place part in {agv}')
            return False

    def floor_robot_change_gripper(self, gripper_type):
        '''
        Change the floor robot's gripper type

        Args:
            gripper_type (str): Gripper type

        Returns:
            bool: True if successful, False otherwise
        '''

        request = ChangeGripper.Request()
        if gripper_type == "TRAY_GRIPPER":
            request.gripper_type = 2
        elif gripper_type == "PART_GRIPPER":
            request.gripper_type = 1
        else:
            raise ValueError('Invalid gripper type')

        future = self._floor_robot_change_gripper_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            self.get_logger().info(
                f'Changed floor robot gripper to {gripper_type}')
            return True
        else:
            self.get_logger().error(
                f'Unable to change floor robot gripper to {gripper_type}')
            return False
    
    def floor_robot_gripper_control(self, enable):
        '''
        Enable/disable floor robot gripper

        Args:
            enable (bool): True to enable, False to disable

        Returns:
            bool: True if successful, False otherwise
        '''

        request = VacuumGripperControl.Request()
        request.enable = enable

        future = self._floor_robot_enable_gripper_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            self.get_logger().info(f'Floor robot gripper control {enable}')
            return True
        else:
            self.get_logger().error(f'Unable to {enable} floor robot gripper')
            return False

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

        self.get_logger().info(f'{self._orders}')

    def _table1_cam_sub_callback(self, msg) -> None:
        '''
        Callback function for table 1 camera subscriber

        Args:
            msg (AdvancedLogicalCameraImage): message received from topic /ariac/sensors/table1_camera/image

        Returns:
            None
        '''

        if len(self._orders) and not self._table1_cam_sub_msg:
            self.get_logger().info(f'{self._node_name}: received camera image from topic /ariac/sensors/table1_camera/image')
            self._table1_cam_sub_msg = True  # only process first message after each order received
            self.get_logger().debug(f'{self._node_name}: received camera image from topic /ariac/sensors/table1_camera/image')

            for tray in msg.tray_poses:
                tray_pose_w = self._multiply_pose(msg.sensor_pose, tray.pose)
                self._tray_poses[tray.id] = KitTrayPose(tray.id, tray_pose_w)
                if tray.id not in self._tables["kts1"]:
                    # self._tables["kts1"] = set()
                    self._tables["kts1"].append(tray.id)
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
            self.get_logger().info(f'{self._node_name}: received camera image from topic /ariac/sensors/table2_camera/image')
            self._table2_cam_sub_msg = True  # only process first message after each order received 
            self.get_logger().debug(f'{self._node_name}: received camera image from topic /ariac/sensors/table2_camera/image')

            for tray in msg.tray_poses:
                tray_pose_w = self._multiply_pose(msg.sensor_pose, tray.pose)
                self._tray_poses[tray.id] = KitTrayPose(tray.id, tray_pose_w)
                if tray.id not in self._tables["kts2"]:
                    # self._tables["kts2"] = set()
                    self._tables["kts2"].append(tray.id)
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
            self.get_logger().info(f'{self._node_name}: received camera image from topic /ariac/sensors/left_bins_camera/image')
            self._left_bins_cam_sub_msg = True  # only process first message after each order received 
            self.get_logger().debug(f'{self._node_name}: received camera image from topic /ariac/sensors/left_bins_camera/image')

            for part in msg.part_poses:
                part_pose_w = self._multiply_pose(msg.sensor_pose, part.pose)
                if part.part.type not in self._part_poses['left_bins']:
                    self._part_poses['left_bins'][part.part.type] = dict()
                if part.part.color not in self._part_poses['left_bins'][part.part.type]:
                    self._part_poses['left_bins'][part.part.type][part.part.color] = list()
                self._part_poses['left_bins'][part.part.type][part.part.color].append(PartPose(part.part, part_pose_w))
            
            self.get_logger().info(f'{self._node_name}: part poses in left bins:\n{self._part_poses["left_bins"]}')

    def _right_bins_cam_sub_callback(self, msg) -> None:
        '''
        Callback function for right bins camera subscriber

        Args:
            msg (AdvancedLogicalCameraImage): message received from topic /ariac/sensors/right_bins_camera/image

        Returns:
            None
        '''

        if not self._right_bins_cam_sub_msg:
            self.get_logger().info(f'{self._node_name}: received camera image from topic /ariac/sensors/right_bins_camera/image')
            self._right_bins_cam_sub_msg = True  # only process first message after each order received 
            self.get_logger().debug(f'{self._node_name}: received camera image from topic /ariac/sensors/right_bins_camera/image')

            for part in msg.part_poses:
                part_pose_w = self._multiply_pose(msg.sensor_pose, part.pose)
                if part.part.type not in self._part_poses['right_bins']:
                    self._part_poses['right_bins'][part.part.type] = dict()
                if part.part.color not in self._part_poses['right_bins'][part.part.type]:
                    self._part_poses['right_bins'][part.part.type][part.part.color] = list()
                self._part_poses['right_bins'][part.part.type][part.part.color].append(PartPose(part.part, part_pose_w))
            
            self.get_logger().info(f'{self._node_name}: part poses in right bins:\n{self._part_poses["right_bins"]}')
    
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

                part_poses = self._part_poses.copy()  # make a copy of part poses
                parts = latest_order.kitting_task.parts
                for part in parts:
                    for bin in part_poses:
                        if part.part.type in part_poses[bin]:
                            if part.part.color in part_poses[bin][part.part.type]:
                                self.get_logger().info(f'{part_poses[bin][part.part.type][part.part.color][0]}')
                                part_poses[bin][part.part.type][part.part.color].pop(0)  # remove part from list
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