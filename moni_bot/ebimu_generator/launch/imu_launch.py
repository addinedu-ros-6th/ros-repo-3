from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    # Node to run imu_raw_publisher
    imu_raw_publisher_node = Node(
        package='ebimu_generator',
        executable='raw_imu_pub',
        name='imu_raw_publisher',
        output='screen'
    )

    # Node to run imu_data_filter
    imu_data_filter_node = Node(
        package='ebimu_generator',
        executable='madgwick_imu_pub',
        name='imu_data_filter',
        output='screen'
    )
    
    imu_publisher_node = Node(
        package='ebimu_generator',
        executable='ebimu_pub',
        name='imu_publisher',
        output='screen'
    )

    return LaunchDescription([
        # Launch imu_raw_publisher first
        imu_raw_publisher_node,

        # Launch imu_data_filter after imu_raw_publisher starts
        imu_data_filter_node,
        
        imu_publisher_node
    ])

# Save this file as imu_launch.py under the launch directory of your ROS package.
