from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # monibot_chatter 노드 실행
    monibot_chatter_node = Node(
        package='monibot_chatter',
        executable='tmp_chatter_publish',
        name='monibot_chatter_node',
        output='screen',
    )
    # monibot_chatter_sub 노드 실행
    monibot_chatter_node_sub = Node(
        package='monibot_chatter',
        executable='tmp_chatter_subcribe',
        name='monibot_chatter_node_sub',
        output='screen',
    )

    return LaunchDescription([
        monibot_chatter_node,
        monibot_chatter_node_sub,
    ])
