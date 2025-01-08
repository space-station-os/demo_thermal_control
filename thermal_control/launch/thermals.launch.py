from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    

    return LaunchDescription([
        Node(
            package='thermal_control',
            executable='coldplate_system',
            name='coldplate_system',
            output='screen',
            
            emulate_tty=True
        ),

        Node(
            package='thermal_control',
            executable='cooling_process',
            name='cooling_process',
            output='screen',
            emulate_tty=True
        ),
    ])
