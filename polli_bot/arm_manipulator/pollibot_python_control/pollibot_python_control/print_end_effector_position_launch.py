import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # 로봇의 패키지 이름 설정
    robot_description_package = 'pollibot_arm_description'
    moveit_config_package = 'pollibot_moveit_config'

    # URDF 파일 경로
    urdf_file_path = os.path.join(
        get_package_share_directory(robot_description_package),
        'urdf',
        'pollibot_arm.urdf'
    )

    # URDF 파일 읽기
    with open(urdf_file_path, 'r') as infp:
        robot_description = infp.read()

    # 로봇 상태 퍼블리셔 노드
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_description}]
    )

    # 조인트 상태 퍼블리셔 노드
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        # 필요에 따라 초기 관절 각도를 설정할 수 있습니다.
        # parameters=[{
        #     'source_list': ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5'],
        #     'initial_positions': {
        #         'joint_1': 0.0,
        #         'joint_2': 0.0,
        #         'joint_3': 0.0,
        #         'joint_4': 0.0,
        #         'joint_5': 0.0,
        #     },
        # }]
    )

    # 엔드 이펙터 위치 출력 노드
    end_effector_position_node = Node(
        package='pollibot_control',  # 실제 패키지 이름으로 변경하세요
        executable='print_end_effector_position.py',
        name='end_effector_position_printer',
        output='screen',
    )

    return LaunchDescription([
        joint_state_publisher_node,
        robot_state_publisher_node,
        end_effector_position_node,
    ])
