from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                namespace = "obstacle", package = "ai_server_pkg",
                executable = "obstacle", output = "screen"
            ),
            Node(
                namespace = "pollination", package = "ai_server_pkg",
                executable = "pollination", output = "screen"
            ),
            Node(
                namespace = "tree_status", package = "ai_server_pkg",
                executable = "tree_status", output = "screen"
            )
        ]
    )