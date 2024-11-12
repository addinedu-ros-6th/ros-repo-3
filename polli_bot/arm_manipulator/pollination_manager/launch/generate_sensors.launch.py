# generate_sensors.launch.py
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Compile and run the C++ pointcloud generator executable
        ExecuteProcess(
            cmd=['g++', 'src/pointcloud_generator.cpp', '-o', 'generator',
                 '-lrealsense2', '-lboost_system', '`pkg-config --cflags --libs opencv4`'],
                 #or just use 'bash', '-c', 'g++ src/pointcloud_generator.cpp -o generator -lrealsense2 -lboost_system `pkg-config --cflags --libs opencv4`'
            output='screen'
        ),
        
        ExecuteProcess(
            cmd=['./generator'],
            cwd='dev_ws',  # Specify the directory if needed
            output='screen'
        ),
        
        # Run the Python script for sending RGB, depth frames via UDP
        ExecuteProcess(
            cmd=['python3', 'src/frame_generator.py'],
            output='screen'
        ),
        
        # ROS2 node for the publishing raw imu
        Node(
            package='pollination_manager',
            executable='ebimu_pub',
            name='ebimu_publisher',
            output='screen'
        )
    ])

