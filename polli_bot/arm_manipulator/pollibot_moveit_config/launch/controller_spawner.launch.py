from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch.substitutions import Command

def generate_launch_description():
    robot_description_content = Command([
        'xacro ', '/home/ask/moveit/src/pollibot_arm_description/urdf/pollibot_arm.urdf.xacro'
    ])

    ros2_control_params = ['/home/ask/moveit/src/pollibot_moveit_config/config/ros2_controllers.yaml']

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description_content
            }]
        ),

        Node(
            package='controller_manager',
            executable='ros2_control_node',
            name='ros2_control_node',
            output='screen',
            parameters=[{
                'robot_description': robot_description_content,
                'ros2_control': ros2_control_params
            }]
        ),

        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['joint_state_broadcaster'],
                    output='screen',
                ),
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['pollibot_arm_controller'],
                    output='screen',
                ),
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['pusher_controller'],
                    output='screen',
                ),
            ],
        ),
    ])


# from launch import LaunchDescription
# from launch_ros.actions import Node
# from launch.actions import TimerAction
# from launch.substitutions import Command

# def generate_launch_description():
#     return LaunchDescription([
#         Node(
#             package='robot_state_publisher',
#             executable='robot_state_publisher',
#             parameters=[{
#                 'robot_description': Command(['xacro ', '/home/ask/moveit/src/pollibot_arm_description/urdf/pollibot_arm.urdf.xacro'])
#             }],
#             output='screen',
#             # parameters=[{'robot_description': Command(['xacro ', '/home/ask/moveit/src/pollibot_arm_description/urdf/pollibot_arm.urdf.xacro'])}],
#             # output='screen',
#         ),

#         Node(
#             package='controller_manager',
#             executable='ros2_control_node',
#             parameters=[{
#                 'robot_description': Command(['xacro ', '/home/ask/moveit/src/pollibot_arm_description/urdf/pollibot_arm.urdf.xacro'])}, 
#             '/home/ask/moveit/src/pollibot_moveit_config/config/ro2_controllers.yaml'],
#             output='screen',
            
#             # parameters=['/home/ask/moveit/src/pollibot_moveit_config/config/controller_manager.yaml'],
#             # output='screen',
#         ),
#         TimerAction(
#             period=5.0,  # Delay in seconds
#             actions=[
#                 Node(
#                     package='controller_manager',
#                     executable='spawner',
#                     arguments=['joint_state_broadcaster'],
#                     output='screen',
#                 ),
#                 Node(
#                     package='controller_manager',
#                     executable='spawner',
#                     arguments=['pollibot_arm_controller'],
#                     output='screen',
#                 ),
#                 Node(
#                     package='controller_manager',
#                     executable='spawner',
#                     arguments=['pusher_controller'],
#                     output='screen',
#                 ),
#             ],
#         ),
#     ])

# from launch import LaunchDescription
# from launch_ros.actions import Node

# def generate_launch_description():
#     return LaunchDescription([
#         Node(
#             package='controller_manager',
#             executable='spawner',
#             arguments=['joint_state_broadcaster'],
#             output='screen',
#         ),
#         Node(
#             package='controller_manager',
#             executable='spawner',
#             arguments=['pollibot_arm_controller'],
#             output='screen',
#         ),
#         Node(
#             package='controller_manager',
#             executable='spawner',
#             arguments=['pusher_controller'],
#             output='screen',
#         ),
#     ])

# combined_moveit_launch.py

# import os
# import yaml
# from launch import LaunchDescription
# from launch_ros.actions import Node
# from ament_index_python.packages import get_package_share_directory
# from moveit_configs_utils import MoveItConfigsBuilder
# from moveit_configs_utils.launches import generate_spawn_controllers_launch

# def generate_launch_description():
#     pkg_share = get_package_share_directory('pollibot_moveit_config')

#     # Load moveit_controllers.yaml
#     controllers_yaml_path = os.path.join(pkg_share, 'config', 'moveit_controllers.yaml')
#     with open(controllers_yaml_path, 'r') as f:
#         controller_manager_params = yaml.safe_load(f)

#     # Build MoveIt configurations
#     moveit_config = MoveItConfigsBuilder(
#         "pollibot",
#         package_name="pollibot_moveit_config"
#     ).to_moveit_configs()


#     # Spawn Controllers
#     spawn_controllers = generate_spawn_controllers_launch(moveit_config)

#     # Move Group Node
#     move_group_node = Node(
#         package="moveit_ros_move_group",
#         executable="move_group",
#         name="move_group",
#         output="screen",
#         parameters=[moveit_config.to_dict()],
#         arguments=["--ros-args", "--log-level", "info"],
#     )

#     return LaunchDescription([
#         spawn_controllers,
#         move_group_node,
#     ])
