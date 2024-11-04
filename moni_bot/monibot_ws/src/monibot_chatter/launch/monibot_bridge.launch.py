import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # YAML 파일 경로 선언
    yaml_file_path = os.path.join(os.getenv('HOME'), 'monibot_ws','src','monibot_chatter','bridge_yaml', 'my_bridge.yaml')

    # domain_bridge 노드 실행
    domain_bridge_node = Node(
        package='domain_bridge',
        executable='domain_bridge',
        name='domain_bridge',
        output='screen',
        arguments=[yaml_file_path],  # YAML 파일 경로 전달
    )


    return LaunchDescription([
        domain_bridge_node,
    ])