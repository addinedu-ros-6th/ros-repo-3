import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from ament_index_python.packages import get_package_share_path, get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

def generate_launch_description():

    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config", 
        default_value="moveit.rviz",
        description="RViz configuration file",
    )

    ros2_control_hardware_type = DeclareLaunchArgument(
        "ros2_control_hardware_type",
        default_value="mock_components",
        description="ROS2 control hardware interface type to use for the launch file -- posiible values are: [mock_components, pollibot_hardware]",
    )

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

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # RViz
    rviz_base = LaunchConfiguration("rviz_config")
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("pollibot_arm_description"), "launch", rviz_base]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="log",
        arguments=[
            "0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "1.0", "world", "base_link"], 
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description,
                    {'use_sim_time': False}],
    )

    # ROS2 controllers
    ros2_controllers_path = os.path.join(
        get_package_share_directory("pollibot_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='controller_manager',
                executable='ros2_control_node',
                output='screen',
                parameters=[moveit_config.robot_description, 
                            ros2_controllers_path,
                            {'use_sim_time': False}],
                remappings=[
                    ("/controller_manager/robot_description", "/robot_description"),
                ]
            ),
        ],
    )

    # 로드할 초기 관절 상태 파일 경로
    # initial_joint_states_path = os.path.join(
    #     get_package_share_directory("pollibot_moveit_config"),
    #     "config",
    #     "initial_positions.yaml"
    # )

    # Joint State Publisher
    # joint_state_publisher_node = Node(
    #     package="joint_state_publisher",
    #     executable="joint_state_publisher",
    #     output="screen",
    #     parameters=[initial_joint_states_path, moveit_config.robot_description
    #     ]
    # )

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
        output="screen",
        parameters=[{'zeros': initial_joint_positions}, 
                    moveit_config.robot_description,
                    {'use_sim_time': False}],
    )

    # joint_state_publisher_node = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     output="screen",
    #     parameters=[{'source_list': initial_joint_positions}],
    # )

    # Load controllers
    load_controllers = []
    for controller in [
        "pollibot_arm_controller",
        "pusher_controller",
        "joint_state_broadcaster",
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner {}".format(controller)],
                shell=True,
                output="screen",
            )
        ]

    pollitask_node = Node(
        package="pollibot_control",
        executable="pollitask_node",
        name="pollitask_node",
        output="screen",
    )

    move_to_target_node = Node(
        package="pollibot_control",
        executable="move_to_target",  # Ensure this matches the C++ executable name
        name="move_to_target",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            # moveit_config.joint_limits,
            {'target_x': 0.14},
            {'target_y': 0.024},
            {'target_z': 0.42},
        ],
    )

    serach_ik_solver_node = Node(
        package="pollibot_control",
        executable="search_IK_solver",
        name="search_IK_solver",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            # moveit_config.joint_limits,
            {'target_x': 0.05},
            {'target_y': 0.22},
            {'target_z': 1.57},
        ],
    )


    # move_to_coordinates_node = TimerAction(
    #     period=5.0,
    #     actions=[
    #         Node(
    #             package="pollibot_control",
    #             executable="move_to_coordinates",
    #             name="move_to_coordinates",
    #             output="screen",
    #             parameters=[
    #                 moveit_config.robot_description,
    #                 moveit_config.robot_description_semantic,
    #                 moveit_config.robot_description_kinematics,
    #                 moveit_config.trajectory_execution,
    #                 moveit_config.planning_pipelines,
    #                 {'target_x': 0.2},  # Replace with desired x coordinate
    #                 {'target_y': 0.0},  # Replace with desired y coordinate
    #                 {'target_z': 0.4},  # Replace with desired z coordinate
    #             ],
    #         )
    #     ]
    # )

    # move_to_coordinates_node = Node(
    #     package="pollibot_control",
    #     executable="move_to_coordinates",
    #     name="move_to_coordinates",
    #     output="screen",
    #     parameters=[
    #         moveit_config.robot_description,
    #         moveit_config.robot_description_semantic,
    #         moveit_config.robot_description_kinematics,
    #         moveit_config.trajectory_execution,
    #         moveit_config.planning_pipelines,
    #         {'target_x': 0.2},  # Replace with desired x coordinate
    #         {'target_y': 0.0},  # Replace with desired y coordinate
    #         {'target_z': 0.4},  # Replace with desired z coordinate
    #     ],
    # )

    # move_to_coordinates_event = RegisterEventHandler(
    #     OnProcessStart(
    #         target_action=move_group_node,
    #         on_start=[move_to_coordinates_node]
    #     )
    # )

    return LaunchDescription(
        [
            ros2_control_hardware_type,
            rviz_config_arg,
            rviz_node,
            static_tf,
            robot_state_publisher,
            move_group_node,
            ros2_control_node,
            serach_ik_solver_node,
            # joint_state_publisher_node,
            # pollitask_node,
            # move_to_target_node,
            # move_to_coordinates_node,
            # move_to_coordinates_event,
        ]
        + load_controllers
    )
