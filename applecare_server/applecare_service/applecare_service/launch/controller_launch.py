#!/usr/bin/env python3
# launch/controller_launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 패키지 디렉토리 가져오기
    pkg_custom_controller = get_package_share_directory('applecare_service')
    pkg_description = get_package_share_directory('turtlebot4_description')  # 터틀봇4 설명 패키지 경로

    # Waypoints 파일 경로
    waypoints_file = os.path.join(pkg_custom_controller, 'config', 'waypoints.yaml')

    # URDF Xacro 파일 경로
    xacro_file = os.path.join(pkg_description, 'urdf', 'standard', 'turtlebot4.urdf.xacro')

    # RViz 설정 파일 경로
    rviz_config_file = os.path.join(pkg_custom_controller, 'config', 'custom_view.rviz')

    # Bridge 구성 파일 경로
    bridge_config_file = os.path.join(pkg_custom_controller, 'config', 'bridge_config.yaml')

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Robot namespace'
    )

    # Set ROS_DOMAIN_ID to 0
    set_domain_id = SetEnvironmentVariable('ROS_DOMAIN_ID', '0')

    # Robot description using xacro
    robot_description_content = Command(['xacro ', xacro_file])
    robot_description = {'robot_description': robot_description_content}

    return LaunchDescription([
        declare_use_sim_time,
        declare_namespace,
        set_domain_id,

        # Robot State Publisher Node
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                robot_description
            ]
        ),

        # Map Publisher Node
        Node(
            package='applecare_service',
            executable='map_publisher.py',
            name='map_publisher',
            output='screen',
            parameters=[{'map_file': os.path.join(pkg_custom_controller, 'config', 'map.yaml')}]
        ),

        # Controller Node
        Node(
            package='applecare_service',
            executable='controller_node.py',
            name='controller_node',
            output='screen',
            parameters=[{'waypoints_file': waypoints_file}]
        ),

        # AppleCare Publisher Node
        Node(
            package='applecare_service',
            executable='applecare_publisher.py',
            name='applecare_publisher',
            output='screen'
        ),

        # AppleCare Subscriber Node
        Node(
            package='applecare_service',
            executable='applecare_subscriber.py',
            name='applecare_subscriber',
            output='screen'
        ),

        # RViz2 Node with custom config
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),

    ])
