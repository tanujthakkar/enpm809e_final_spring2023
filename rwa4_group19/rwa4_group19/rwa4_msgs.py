#! /usr/bin/env python3

# ENPM 809E - RWA4
# Author(s): Tanuj Thakkar (117817539), Aneesh Dandime (118418506)
# Email(s): tanuj@umd.edu, aneeshd@umd.edu

from geometry_msgs.msg import Pose

class Part:
    """
    Class to represent a part

    Constants:
        COLOR (dict): dictionary mapping color number to color name
        TYPE (dict): dictionary mapping type number to type name

    Attributes:
        color (int): color of the part
        type (int): type of the part

    Methods:
        __str__(): returns a string representation of the part
        from_msg(msg): returns a Part object from a ROS message
    """

    COLOR = {0 : "RED",
             1 : "GREEN",
             2 : "BLUE",
             3 : "ORANGE",
             4 : "PURPLE",}

    TYPE = {10 : "BATTERY",
            11 : "PUMP",
            12 : "SENSOR",
            13 : "REGULATOR"}

    def __init__(self, color : int, type : int) -> None:
        self.color = color
        self.type = type
    
    def __str__(self) -> str:
        return f"Part\n color: {self.COLOR[self.color]}\n type: {self.TYPE[self.type]}"

    @classmethod
    def from_msg(cls, msg) -> "Part":
        return cls(msg.color, msg.type)


class KittingPart:
    """
    Class to represent a kitting part

    Constants:
        QUADRANT (dict): dictionary mapping quadrant number to quadrant name

    Attributes:
        quadrant (int): quadrant of the part
        part (Part): part object

    Methods:
        __str__(): returns a string representation of the kitting part
        from_msg(msg): returns a KittingPart object from a ROS message
    """

    QUADRANT = {1 : "QUADRANT1",
                2 : "QUADRANT2",
                3 : "QUADRANT3",
                4 : "QUADRANT4"}

    def __init__(self, quadrant : int, part : Part) -> None:
        self.quadrant = quadrant
        self.part = part

    def __str__(self) -> str:
        return f"KittingPart\n quadrant: {self.QUADRANT[self.quadrant]}\n part:\n{self.part}"

    @classmethod
    def from_msg(cls, msg) -> "KittingPart":
        return cls(msg.quadrant, Part.from_msg(msg.part))


class KittingTask:
    """
    Class to represent a kitting task

    Constants:
        DESTINATION (dict): dictionary mapping destination number to destination name

    Attributes:
        agv_number (int): agv number
        tray_id (int): tray id
        destination (int): destination of the tray
        parts (list): list of kitting parts

    Methods:
        __str__(): returns a string representation of the kitting task
        from_msg(msg): returns a KittingTask object from a ROS message
    """


    DESTINATION = {0 : "KITTING",
                   1 : "ASSEMBLY_FRONT",
                   2 : "ASSEMBLY_BACK",
                   3 : "WAREHOUSE"}

    def __init__(self, agv_number : int, tray_id : int, destination : int, parts : list) -> None:
        self.agv_number = agv_number
        self.tray_id = tray_id
        self.destination = destination
        self.parts = parts

    def __str__(self) -> str:
        s = f"KittingTask\n agv_number: {self.agv_number}\n tray_id: {self.tray_id}\n destination: {self.DESTINATION[self.destination]}\n parts:\n"
        for part in self.parts:
            s += f"{part}\n"
        return s

    @classmethod
    def from_msg(cls, msg) -> "KittingTask":
        kitting_parts = list()
        for kitting_part in msg.parts:
            kitting_parts.append(KittingPart.from_msg(kitting_part))
        return cls(msg.agv_number, msg.tray_id, msg.destination, kitting_parts)


class Order:
    """
    Class to represent an order

    Constants:
        TASK (dict): dictionary mapping task number to task name

    Attributes:
        id (str): id of the order
        type (int): type of the order
        priority (bool): priority of the order
        kitting_task (KittingTask): kitting task object

    Methods:
        __str__(): returns a string representation of the order
        from_msg(msg): returns an Order object from a ROS message
    """

    TASK = {0 : "KITTING",
            1 : "ASSEMBLY",
            2 : "COMBINED"}

    def __init__(self, id : str, type : int, priority : bool, kitting_task : KittingTask=None) -> None:
        self.id = id
        self.type = type
        self.priority = priority
        self.kitting_task = kitting_task

    def __str__(self) -> str:
        s = f"Order\n id: {self.id}\n type: {self.TASK[self.type]}\n priority: {self.priority}\n"
        if self.kitting_task is not None:
            s += f"kitting_task:\n{self.kitting_task}"
        return s

    @classmethod
    def from_msg(cls, msg) -> "Order":
        kitting_task = None
        if msg.kitting_task is not None:
            kitting_task = KittingTask.from_msg(msg.kitting_task)
        return cls(msg.id, msg.type, msg.priority, kitting_task)


class KitTrayPose:
    """
    Class to represent a kit tray pose
    
    Attributes:
        id (int): id of the tray
        pose (Pose): pose of the tray

    Methods:
        __str__(): returns a string representation of the kit tray pose
        from_msg(msg): returns a KitTrayPose object from a ROS message
    """


    def __init__(self, id : int, pose : Pose):
        self.id = id
        self.pose = pose
    
    def __str__(self) -> str:
        # return f"KitTrayPose\n id: {self.id}\n pose:\n{self.pose}"
        s = f'\nTray: \
              \n - id: {self.id} \
              \n - pose: \
              \n   - position: [{self.pose.position.x, self.pose.position.y, self.pose.position.z}] \
              \n   - orientation: [{self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w}]'
        return s

    @classmethod
    def from_msg(cls, msg) -> "KitTrayPose":
        return cls(msg.id, msg.pose)


class PartPose:
    """
    Class to represent a part pose

    Attributes:
        part (Part): part object
        pose (Pose): pose of the part

    Methods:
        __str__(): returns a string representation of the part pose
        from_msg(msg): returns a PartPose object from a ROS message
    """

    def __init__(self, part : Part, pose : Pose):
        self.part = part
        self.pose = pose
    
    def __str__(self) -> str:
        # return f"PartPose\n part:\n{self.part}\n pose:\n{self.pose}"
        s = f'\nPart: \
              \n - {Part.COLOR[self.part.color]} {Part.TYPE[self.part.type]} \
              \n - pose: \
              \n   - position: [{self.pose.position.x, self.pose.position.y, self.pose.position.z}] \
              \n   - orientation: [{self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w}]'
        return s

    @classmethod
    def from_msg(cls, msg) -> "PartPose":
        return cls(Part.from_msg(msg.part), msg.pose)