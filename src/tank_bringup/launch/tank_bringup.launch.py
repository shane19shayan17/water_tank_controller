from launch import LaunchDescription
from launch_ros.actions import Node

# get pkg path
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('water_tank_interface'),
        'params',
        'controller.yaml'
    )
    return LaunchDescription([
        Node(
            package='controller',
            executable='controller',
            name='controller',
            parameters=[
                params_file
            ]
        ),
        Node(
            package='simulator',
            executable='tank_sim',
            name='simulator'
        ),
    ])