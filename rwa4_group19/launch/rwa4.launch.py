import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    '''
    Function that returns a LaunchDescription object

    Returns:
        LaunchDescription: LaunchDescription object
    '''
    launch_description = LaunchDescription()
    
    rwa4_node = Node(
        package="rwa4_group19",
        executable="rwa4")
    
    launch_description.add_action(rwa4_node)
    return launch_description
