# Copyright 2022 Clearpath Robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    pkg_pollibot_slam_cartographer = get_package_share_directory('slam_cartographer')
    pkg_cartographer_ros = get_package_share_directory('cartographer_ros')

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    namespace = LaunchConfiguration('namespace', default='')
    cartographer_config_dir = LaunchConfiguration(
        'cartographer_config_dir',
        default=os.path.join(pkg_pollibot_slam_cartographer, 'config')
    )
    configuration_basename = LaunchConfiguration(
        'configuration_basename', default='pollibot_2d.lua'
    )
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

    # Declare launch arguments for flexibility
    declared_arguments = [
        DeclareLaunchArgument(
            'use_sim_time', default_value='false', description='Use simulation time if true'
        ),
        DeclareLaunchArgument(
            'namespace', default_value='', description='Robot namespace'
        ),
        DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=PathJoinSubstitution([pkg_pollibot_slam_cartographer, 'config']),
            description='Directory with Cartographer configuration files'
        ),
        DeclareLaunchArgument(
            'configuration_basename',
            default_value='pollibot_2d.lua',
            description='Configuration file for Cartographer'
        ),
        DeclareLaunchArgument(
            'publish_period_sec',
            default_value='1.0',
            description='Publishing period for the occupancy grid in seconds'
        ),
    ]

    # Remappings
    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
        ('/scan', 'scan'),
        ('/odom', 'odom')
    ]

    # Cartographer Node
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', configuration_basename
        ],
        remappings=remappings,
    )

    # Delayed Cartographer Node
    delayed_cartographer_node = TimerAction(
        period=5.0,  # Delay in seconds
        actions=[cartographer_node]
    )

    # Occupancy Grid Node
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'resolution': 0.05,
                'publish_period_sec': publish_period_sec,
            }
        ],
        remappings=remappings,
    )

    # Static Transform Publisher
    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )   
    
    odom_baselink_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_baselink',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        output='screen'
    )



    # RViz Node
    rviz_config_dir = os.path.join(pkg_pollibot_slam_cartographer, 'rviz', 'cartographer.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Define Launch Description
    ld = LaunchDescription(declared_arguments)
    ld.add_action(static_transform_publisher)  # Launch the static transform publisher
    ld.add_action(odom_baselink_publisher)  # Launch the static transform publisher
    ld.add_action(delayed_cartographer_node)  # Add delayed Cartographer node
    ld.add_action(occupancy_grid_node)  # Launch occupancy grid node
    ld.add_action(rviz_node)  # Launch RViz

    return ld
