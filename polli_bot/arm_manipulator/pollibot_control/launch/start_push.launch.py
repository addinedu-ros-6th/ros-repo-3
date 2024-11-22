import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Paths to URDF and SRDF files
    package_moveit_config_path = get_package_share_directory('pollibot_moveit_config')
    package_arm_description_path = get_package_share_directory('pollibot_arm_description')
    urdf_path = os.path.join(package_arm_description_path, 'urdf', 'pollibot_arm.urdf')
    srdf_path = os.path.join(package_moveit_config_path, 'config', 'pollibot.srdf')

    # Read URDF and SRDF files
    with open(urdf_path, 'r') as urdf_file:
        robot_description = urdf_file.read()
    with open(srdf_path, 'r') as srdf_file:
        robot_description_semantic = srdf_file.read()

    return LaunchDescription([
        Node(
            package='pollibot_control',
            executable='start_push',
            name='start_push',
            output='screen',
            parameters=[
                {'robot_description': robot_description},
                {'robot_description_semantic': robot_description_semantic},
            ],
        ),
    ])
