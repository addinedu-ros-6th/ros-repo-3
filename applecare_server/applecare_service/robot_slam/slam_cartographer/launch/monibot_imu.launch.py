# Copyright 2019 Open Source Robotics Foundation, Inc.
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
#
# Author: Darby Lim

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    monibot_cartographer_prefix = get_package_share_directory('slam_cartographer')
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', default=os.path.join(
                                                  monibot_cartographer_prefix, 'config'))
    cartographer_src_dir = LaunchConfiguration('cartographer_src_dir', default=os.path.join(
                                                  monibot_cartographer_prefix, 'src'))
    configuration_basename = LaunchConfiguration('configuration_basename',
                                                 default='monibot_imu_2d.lua') #change this to pollibot when using pollibot

    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

    rviz_config_dir = os.path.join(get_package_share_directory('slam_cartographer'),
                                   'rviz', 'cartographer_imu.rviz')
 
    return LaunchDescription([ 
        DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=cartographer_config_dir,
            description='Full path to config file to load'),
        DeclareLaunchArgument(
            'configuration_basename',
            default_value=configuration_basename,
            description='Name of lua file for cartographer'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        # initialize odom_filtered first and then execute cartographer_ros
        # Node(
        #     package='slam_cartographer',
        #     executable='ekf_odom.py',
        #     name='ekf_odom_filter',
        #     output='screen',
        #     parameters=[{'use_sim_time': use_sim_time}],
        #     arguments=['-configuration_directory', cartographer_src_dir],
            
        # ),

        # cartographer_node delayed version
        # TimerAction(
        #     period=3.0,  # Delay in seconds
        #     actions=[
        #         Node(
        #             package='cartographer_ros',
        #             executable='cartographer_node',
        #             name='cartographer_node',
        #             output='screen',
        #             parameters=[{'use_sim_time': use_sim_time},
        #                         {'queue_size': 100}  # determines how many messages can be stored if they arrive faster than they can be processed
        #                         ],
        #             arguments=['-configuration_directory', cartographer_config_dir,
        #                     '-configuration_basename', configuration_basename],
        #             # remappings=[('/odom', '/base_controller/odom')]  # Remap /odom to /base_controller/odom
        #             remappings=[('/odom', '/odom_filter')]  # Remap /odom to /odom_filtered

        #         )
        #     ]
        # ),
        
        # cartographer_node normal version
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', configuration_basename],
            remappings=[('/odom', '/base_controller/odom')]  # Remap /odom to /base_controller/odom
            # remappings=[('/odom', '/odom_filter')]  # Remap /odom to /odom_filter

        ),

        #tf node
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_broadcaster',
            arguments=['0', '0', '0.034', '0', '0', '0', 'map', 'odom'],
            output='screen'
        ),

        DeclareLaunchArgument(
            'resolution',
            default_value=resolution,
            description='Resolution of a grid cell in the published occupancy grid'),

        DeclareLaunchArgument(
            'publish_period_sec',
            default_value=publish_period_sec,
            description='OccupancyGrid publishing period'),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/occupancy_grid.launch.py']),
        #     launch_arguments={'use_sim_time': use_sim_time, 'resolution': resolution,
        #                       'publish_period_sec': publish_period_sec}.items(),
        # ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/occupancy_grid.launch.py']),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'resolution': resolution,
                'publish_period_sec': publish_period_sec
                # Add this remapping to change the odom frame
            }.items(),
        ),


        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])
