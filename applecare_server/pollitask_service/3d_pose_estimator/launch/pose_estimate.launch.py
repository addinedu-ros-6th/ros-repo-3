from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os

def generate_launch_description():

    # --------------------------------------------
    # receive sensors first before 3d pose estimation
    
    workspace_dir = os.path.join(os.getenv('HOME'), 'ros-repo-3/applecare_server/pollitask_service/3d_pose_estimator')

    return LaunchDescription([
        # Compile the C++ pointcloud generator executable
        ExecuteProcess(
            cmd=['bash', '-c', 'g++ src/pointcloud_processor.cpp -o pointcloud_processor -lrealsense2 -lboost_system `pkg-config --cflags --libs opencv4`'],
            cwd=workspace_dir,
            output='screen'
        ),
        
        # Run the compiled C++ executable (assuming generator is in 'ksm' after compilation)
        ExecuteProcess(
            cmd=[os.path.join(workspace_dir, './pointcloud_processor')],
            output='screen'
        ),
        
        # Run the Python script for sending frames via UDP
        ExecuteProcess(
            cmd=['python3', os.path.join(workspace_dir, 'src', 'frame_sub.py')],
            output='screen'
        ),
        
        # Launch the ROS2 node for publishing raw IMU data
        Node(
            package='3d_pose_estimator',
            executable='raw_ebimu_sub.py',
            name='raw_ebimu_subscriber',
            output='screen'
        )
        # --------------------------------------------------------------------
        # now initiate nodes for pose estimation (VIO)
        
        
    ])

