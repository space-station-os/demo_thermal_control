from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_file= os.path.join(
        get_package_share_directory('thermal_control'),
        'config',
        'thermal_network.yaml'
    )

    return LaunchDescription([
        Node(
            package='thermal_control',
            executable='thermal_node',
            name='thermal_solver_node',
            output='screen',
            parameters=[config_file],
            emulate_tty=True
        ),

        
    ])
