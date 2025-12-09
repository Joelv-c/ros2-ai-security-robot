"""
Launch file for the Robot Brain package.
This script initializes the distributed system by starting the Vision Node
and the Driver Node simultaneously.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Generate the launch description for the autonomous security robot.
    """
    
    # 1. Define the Vision Node
    # This node handles camera input and YOLOv8 inference.
    vision_node = Node(
        package='robot_brain',
        executable='vision_node',
        name='vision_node',
        output='screen',
        emulate_tty=True
    )

    # 2. Define the Driver Node 
    # This node handles motor control.
    driver_node = Node(
        package='robot_brain',
        executable='driver_node',
        name='driver_node',
        output='screen',
        emulate_tty=True
    )

    # 3. Define the Keyboard Control Node
     keyboard_node = Node(
         package='robot_brain',
         executable='keyboard_node',
         name='keyboard_node',
         output='screen',
         prefix='xterm -e'  # Opens in a new terminal window for input
     )

    return LaunchDescription([
        vision_node,
        driver_node,
        keyboard_node, 
    ])