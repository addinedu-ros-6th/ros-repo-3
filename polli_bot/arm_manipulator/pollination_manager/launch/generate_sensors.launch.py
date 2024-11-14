from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os

def generate_launch_description():
    workspace_dir = os.path.join(os.getenv('HOME'), 'ksm/src/pollination_manager')

    return LaunchDescription([
        # Compile the C++ pointcloud generator executable
        ExecuteProcess(
            cmd=['bash', '-c', 'g++ src/pointcloud_generator.cpp -o pointcloud_generator -lrealsense2 -lboost_system `pkg-config --cflags --libs opencv4`'],
            cwd=workspace_dir,
            output='screen'
        ),
        
        # Run the compiled C++ executable (assuming generator is in 'ksm' after compilation)
        ExecuteProcess(
            cmd=[os.path.join(workspace_dir, './pointcloud_generator')],
            output='screen'
        ),
        
        # Run the Python script for sending frames via UDP
        ExecuteProcess(
            cmd=['python3', os.path.join(workspace_dir, 'src', 'frame_generator.py')],
            output='screen'
        ),
        
        # Launch the ROS2 node for publishing raw IMU data
        Node(
            package='pollination_manager',
            executable='raw_ebimu_pub.py',
            name='raw_ebimu_publisher',
            output='screen'
        )
    ])

