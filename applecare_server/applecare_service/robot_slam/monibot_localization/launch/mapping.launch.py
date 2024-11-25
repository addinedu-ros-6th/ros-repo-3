from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    ekf_filter = Node(
        package='monibot_localization',
        executable='ekf_filter',
        name='ekf_odom_filter',
        output='screen',
        parameters=[os.path.join(
            get_package_share_directory("monibot_localization"), 'params', 'params.yaml')],
    )
    
    scan_matching = Node(
        package='monibot_localization',
        executable='scan_matching',
        name='scan_to_map_node',
        output='screen',
        parameters=[os.path.join(
            get_package_share_directory("monibot_localization"), 'params', 'params.yaml')],
    )

    simple_amcl = Node(
        package='monibot_localization',
        executable='simple_amcl',
        name='simple_amcl_node',
        output='screen',
        parameters=[os.path.join(
            get_package_share_directory("monibot_localization"), 'params', 'params.yaml')],
        remappings=[('/odom', '/odom_filter')],
    )

    static_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom'],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(
            get_package_share_directory('monibot_localization'),
            'rviz', 'localization_mapping.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    return LaunchDescription([
        ekf_filter,
        scan_matching,
        simple_amcl,
        # static_tf_publisher,
        rviz_node
    ])
