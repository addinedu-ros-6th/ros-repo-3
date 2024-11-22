#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import subprocess
import threading

class PollitaskNode(Node):
    def __init__(self):
        super().__init__('pollitask_node')
        self.subscription = self.create_subscription(
            Bool,
            'pollibot/pollination_start',
            self.pollination_start_callback,
            10)
        self.publisher = self.create_publisher(Bool, 'pollibot/pollination_complete', 10)
        self.launch_process = None
        self.launch_lock = threading.Lock()
        self.get_logger().info("PollitaskNode initialized and listening to 'pollibot/pollination_start'")

    def pollination_start_callback(self, msg):
        if msg.data:
            self.get_logger().info("Received 'pollination_start' signal. Launching 'move_to_ready_pose.launch.py'")
            with self.launch_lock:
                if self.launch_process is None or self.launch_process.poll() is not None:
                    self.get_logger().info("Launching 'plan_execution.launch.py'")
                    # 실행할 Launch 파일의 전체 경로를 지정
                    launch_file_path = os.path.join(
                        get_package_share_directory('pollibot_moveit_config'),
                        'launch',
                        'plan_execution.launch.py'
                    )
                    if not os.path.exists(launch_file_path):
                        self.get_logger().error(f"Launch file not found: {launch_file_path}")
                        return

                    # Start the Launch file
                    self.launch_process = subprocess.Popen(
                        ['ros2', 'launch', 'pollibot_control', 'move_to_ready_pose.launch.py'],
                        # ['ros2', 'launch', 'pollibot_moveit_config', 'plan_execution.launch.py'],
                        stdout=subprocess.PIPE,
                        stderr=subprocess.PIPE
                    )
                    # Start a thread to monitor the Launch process
                    monitor_thread = threading.Thread(target=self.monitor_launch_process)
                    monitor_thread.start()
                else:
                    self.get_logger().warn("Launch process is already running.")

    def monitor_launch_process(self):
        stdout, stderr = self.launch_process.communicate()
        if self.launch_process.returncode == 0:
            self.get_logger().info("Launch file executed successfully.")
            # Publish completion message
            completion_msg = Bool()
            completion_msg.data = True
            self.publisher.publish(completion_msg)
            self.get_logger().info("Published 'pollination_complete' signal.")
        else:
            self.get_logger().error(f"Launch file failed with return code {self.launch_process.returncode}.")
            self.get_logger().error(stderr.decode())
        # Reset the process
        with self.launch_lock:
            self.launch_process = None

def main(args=None):
    rclpy.init(args=args)
    node = PollitaskNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down PollitaskNode')
    finally:
        # If launch process is still running, terminate it
        with node.launch_lock:
            if node.launch_process and node.launch_process.poll() is None:
                node.launch_process.terminate()
                node.get_logger().info("Terminated launch process.")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
