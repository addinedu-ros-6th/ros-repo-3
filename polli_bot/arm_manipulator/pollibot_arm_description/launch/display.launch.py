import os
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # 기본 경로 설정
    pollibot_path = get_package_share_path('pollibot_arm_description')
    default_model_path = pollibot_path / 'urdf/pollibot_arm.urdf'
    default_rviz_config_path = pollibot_path / 'rviz/pollibot.rviz'

    # Launch Arguments
    gui_arg = DeclareLaunchArgument(name='gui', default_value='true', choices=['true', 'false'],
                                    description='Flag to enable joint_state_publisher_gui')
    model_arg = DeclareLaunchArgument(name='model', default_value=str(default_model_path),
                                      description='Absolute path to robot urdf file')
    # rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path),
    #                                  description='Absolute path to rviz config file')

    # robot_description 설정 (URDF 파일 경로)
    robot_description = ParameterValue(Command(['cat ', LaunchConfiguration('model')]), value_type=str  )

    # robot_state_publisher 노드
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # 초기 관절 위치 설정
    initial_joint_positions = {
        'joint_1': 2.35619,
        'joint_2': 2.00713,
        'joint_3': 2.35619,
        'joint_4': 2.35619
    }
    # joint_state_publisher 및 joint_state_publisher_gui 노드
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'zeros': initial_joint_positions}],
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        parameters=[{'zeros': initial_joint_positions}],
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "1.0", "world", "base_link"],
        output="log"
    )

    # RViz 노드
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',
    #     arguments=['-d', LaunchConfiguration('rvizconfig')],
    # )

    return LaunchDescription([
        gui_arg,
        model_arg,
        # rviz_arg,
        static_tf_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        # rviz_node,
    ])
