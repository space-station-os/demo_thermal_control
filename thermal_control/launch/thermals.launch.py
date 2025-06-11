from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    

    return LaunchDescription([
        Node(
            package='thermal_control',
            executable='coolant',
            name='internal_coolant',
            output='screen',
            
            emulate_tty=True
        ),

        Node(
            package='thermal_control',
            executable='external_loop',
            name='external_loop',
            output='screen',
            emulate_tty=True
        ),
    ])
