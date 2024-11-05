#!/usr/bin/env python3
# launch/waypoint_setter_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package share directory
    pkg_custom_controller = get_package_share_directory('applecare_service')

    # Map file
    map_file = os.path.join(pkg_custom_controller, 'config', 'map.yaml')

    return LaunchDescription([
        # Waypoint Setter Node
        Node(
            package='applecare_service',
            executable='waypoint_setter.py',
            name='waypoint_setter',
            output='screen',
            parameters=[{'map_file': map_file}]
        )
    ])
