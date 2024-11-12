from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch the pollitask subscriber nodes
        Node(
            package='3d_pose_estimator',
            executable='pointcloud_processor',
            name='pointcloud_processor',
            output='screen'
        ),
        Node(
            package='3d_pose_estimator',
            executable='depthcam_subscriber',
            name='depthcam_sub',
            output='screen'
        ),
        Node(
            package='3d_pose_estimator',
            executable='raw_ebimu_subscriber',
            name='raw_imu_sub',
            output='screen'
        ),
    ])
