from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # 현재 패키지 경로 가져오기
    package_dir = os.path.dirname(os.path.realpath(__file__))

    return LaunchDescription([
        Node(
            package='applecare_gui_pkg',
            executable='gui_main',
            name='gui_main',
            output='screen',
            parameters=[]
        )
    ])
