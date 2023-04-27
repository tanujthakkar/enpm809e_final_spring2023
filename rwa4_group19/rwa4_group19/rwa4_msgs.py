#! /usr/bin/env python3

# ENPM 809E - RWA4
# Author(s): Tanuj Thakkar (117817539), Aneesh Dandime (118418506)
# Email(s): tanuj@umd.edu, aneeshd@umd.edu

from geometry_msgs.msg import Pose

class Part:
    _COLOR = {0 : "RED",
              1 : "GREEN",
              2 : "BLUE",
              3 : "ORANGE",
              4 : "PURPLE",}

    _TYPE = {10 : "BATTERY",
             11 : "PUMP",
             12 : "SENSOR",
             13 : "REGULATOR"}

    def __init__(self, color : int, type : int) -> None:
        self.color = color
        self.type = type
    
    def __str__(self) -> str:
        return f"Part\n color: {self._COLOR[self.color]}\n type: {self._TYPE[self.type]}"

    @classmethod
    def from_msg(cls, msg) -> "Part":
        return cls(msg.color, msg.type)


class KittingPart:
    _QUADRANT = {1 : "QUADRANT1",
                 2 : "QUADRANT2",
                 3 : "QUADRANT3",
                 4 : "QUADRANT4"}

    def __init__(self, quadrant : int, part : Part) -> None:
        self.quadrant = quadrant
        self.part = part

    def __str__(self) -> str:
        return f"KittingPart\n quadrant: {self._QUADRANT[self.quadrant]}\n part:\n{self.part}"

    @classmethod
    def from_msg(cls, msg) -> "KittingPart":
        return cls(msg.quadrant, Part.from_msg(msg.part))


class KittingTask:
    _DESTINATION = {0 : "KITTING",
                    1 : "ASSEMBLY_FRONT",
                    2 : "ASSEMBLY_BACK",
                    3 : "WAREHOUSE"}

    def __init__(self, agv_number : int, tray_id : int, destination : int, parts : list) -> None:
        self.agv_number = agv_number
        self.tray_id = tray_id
        self.destination = destination
        self.parts = parts

    def __str__(self) -> str:
        s = f"KittingTask\n agv_number: {self.agv_number}\n tray_id: {self.tray_id}\n destination: {self._DESTINATION[self.destination]}\n parts:\n"
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
    _TASK = {0 : "KITTING",
             1 : "ASSEMBLY",
             2 : "COMBINED"}

    def __init__(self, id : str, type : int, priority : bool, kitting_task : KittingTask=None) -> None:
        self.id = id
        self.type = type
        self.priority = priority
        self.kitting_task = kitting_task

    def __str__(self) -> str:
        s = f"Order\n id: {self.id}\n type: {self._TASK[self.type]}\n priority: {self.priority}\n"
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
    def __init__(self, id : int, pose : Pose):
        self.id = id
        self.pose = pose
    
    def __str__(self) -> str:
        return f"KitTrayPose\n id: {self.id}\n pose:\n{self.pose}"
    
    @classmethod
    def from_msg(cls, msg) -> "KitTrayPose":
        return cls(msg.id, msg.pose)