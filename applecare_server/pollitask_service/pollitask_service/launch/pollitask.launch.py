from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch the pollitask subscriber node
        Node(
            package='pollitask_service',
            executable='depthcam_subscriber',
            name='depthcam_sub',
            output='screen'
        ),
        Node(
            package='pollitask_service',
            executable='ebimu_subscriber',
            name='ebimu_sub',
            output='screen'
        ),
    ])
