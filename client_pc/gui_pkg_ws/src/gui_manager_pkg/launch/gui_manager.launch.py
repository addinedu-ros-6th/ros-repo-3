from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gui_manager_pkg',
            executable='gui_manager',
            name='gui_manager_1',  # 유니크한 이름을 설정
            output='screen',
            parameters=[]
        ),
    ])
