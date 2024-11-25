from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Declare Launch Arguments
    map_yaml_file_arg = DeclareLaunchArgument('map', 
        default_value=os.path.join(
            get_package_share_directory('monibot_localization'),
            'map',
            'cartographer_final_modified.yaml'
        ),
        description='Full path to the map YAML file'
    )

    use_sim_time_arg = DeclareLaunchArgument('use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )

    # Launch Configurations
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')

    ekf_filter = Node(
        package='monibot_localization',
        executable='ekf_filter',
        name='ekf_odom_filter',
        output='log',
        parameters=[
            os.path.join(
                get_package_share_directory("monibot_localization"),
                'params',
                'params.yaml'
            ),
            {'use_sim_time': use_sim_time}
        ],
    )

    map_publisher = Node(
        package='monibot_localization',
        executable='map_publisher',
        name='map_publisher',
        output='log',
        parameters=[{'map_yaml_file': map_yaml_file}]
    )

    custom_amcl = Node(
        package='monibot_localization',
        executable='custom_amcl',
        name='custom_amcl_node',
        output='log',
        parameters=[
            os.path.join(
                get_package_share_directory("monibot_localization"),
                'params',
                'params.yaml'
            ),
            {'use_sim_time': use_sim_time}
        ],
        remappings=[('/odom', '/odom_filter')],
    )

    # nav2 amcl
    amcl_node = Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            parameters=[{
                'use_sim_time': False,
                'min_particles': 500,
                'max_particles': 2000,
                'kld_err': 0.05,
                'kld_z': 0.99,
                'odom_alpha1': 0.2,
                'odom_alpha2': 0.2,
                'odom_alpha3': 0.2,
                'odom_alpha4': 0.2,
                'laser_min_range': 0.1,
                'laser_max_range': 30.0,
                'laser_z_hit': 0.95,
                'laser_z_short': 0.1,
                'laser_z_max': 0.05,
                'laser_z_rand': 0.05,
                'laser_sigma_hit': 0.2,
            }]
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(
            get_package_share_directory('monibot_localization'),
            'rviz',
            'localization.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    return LaunchDescription([
        map_yaml_file_arg,
        use_sim_time_arg,
        ekf_filter,
        map_publisher,
        # custom_amcl,
        amcl_node,
        rviz_node,
    ])
