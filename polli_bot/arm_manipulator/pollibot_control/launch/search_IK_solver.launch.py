import os
import yaml
from yaml import SafeLoader
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Declare hardware type argument
    ros2_control_hardware_type = DeclareLaunchArgument(
        'ros2_control_hardware_type',
        default_value='mock_components',
        description='ROS2 control hardware interface type to use for the launch file -- possible values are: [mock_components, pollibot_hardware]',
    )

    # Configure MoveIt
    moveit_config = (
        MoveItConfigsBuilder("pollibot")
        .robot_description(
            file_path="config/pollibot.urdf.xacro",
            mappings={
                "ros2_control_hardware_type": LaunchConfiguration(
                    "ros2_control_hardware_type"
                )
            }
        )
        .robot_description_semantic(file_path="config/pollibot.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # Paths for additional configuration files
    package_moveit_config_path = get_package_share_directory('pollibot_moveit_config')
    package_arm_description_path = get_package_share_directory('pollibot_arm_description')
    urdf_path = os.path.join(package_arm_description_path, 'urdf', 'pollibot_arm.urdf')
    srdf_path = os.path.join(package_moveit_config_path, 'config', 'pollibot.srdf')
    kinematics_yaml_path = os.path.join(package_moveit_config_path, 'config', 'kinematics.yaml')
    moveit_controllers_yaml_path = os.path.join(package_moveit_config_path, 'config', 'moveit_controllers.yaml')

    # Read kinematics and controllers YAML files
    with open(kinematics_yaml_path, 'r') as f:
        kinematics_yaml = yaml.load(f, Loader=SafeLoader)
    with open(moveit_controllers_yaml_path, 'r') as f:
        moveit_controllers_yaml = yaml.load(f, Loader=SafeLoader)

    # Initial joint positions
    initial_joint_positions = {
        'joint_1': 2.35619,
        'joint_2': 2.00713,
        'joint_3': 2.35619,
        'joint_4': 2.35619
    }

    # Nodes
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[moveit_config.robot_description, {'use_sim_time': False}],
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output="screen",
        parameters=[
            {'zeros': initial_joint_positions}, 
            moveit_config.robot_description, 
            {'use_sim_time': False}
        ],
    )

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[moveit_config.to_dict(), {'use_sim_time': False}],
    )

    move_to_target_node = Node(
        package='pollibot_control',
        executable='move_to_target',
        output='screen',
        parameters=[
            moveit_config.to_dict(), 
            kinematics_yaml, 
            moveit_controllers_yaml, 
            {'use_sim_time': False}
        ],
    )

    return LaunchDescription([
        ros2_control_hardware_type,
        robot_state_publisher_node,
        # joint_state_publisher_node,
        move_group_node,
        move_to_target_node,
    ])
