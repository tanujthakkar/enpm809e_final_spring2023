import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    '''
    Function that returns a LaunchDescription object

    Returns:
        LaunchDescription: LaunchDescription object
    '''
    launch_description = LaunchDescription()

    config = os.path.join(get_package_share_directory("rwa4_group19"), 
                          'config', 
                          'order.yaml')
    
    rwa4_node = Node(
        package="rwa4_group19",
        executable="rwa4",
        parameters=[config],
        output="screen",
    )
    
    launch_description.add_action(rwa4_node)
    return launch_description
