import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
import matplotlib.pyplot as plt
from laser_geometry import LaserProjection
from rclpy.qos import QoSProfile, QoSReliabilityPolicy


class GraphBasedSLAMNode(Node):
    def __init__(self):
        super().__init__('graph_based_slam')
        
        # Initialize pose graph
        self.pose_graph = []
        self.current_pose = np.array([0.0, 0.0, 0.0])  # Initial pose (x, y, theta)
        self.pose_graph.append(self.current_pose)
        
        # Initialize edge list (store relative transformations)
        self.edges = []
        
        # Set up QoS profile to match the LiDAR's QoS
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT

        # Subscribe to /scan topic for LIDAR data
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, qos_profile)
        
        # Subscribe to /odom_filtered topic for odometry data
        self.create_subscription(Odometry, '/odom_filter', self.odometry_callback, 10)

        # Optionally, you can subscribe to IMU data for orientation if needed
        # self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        
        # Initialize LIDAR projection (converting scan to 2D coordinates)
        self.laser_proj = LaserProjection()
        
        # Odometry data (initial pose)
        self.odom_pose = None

    def lidar_callback(self, msg: LaserScan):
        # Convert LIDAR scan to points in Cartesian coordinates
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        x_points = ranges * np.cos(angles)
        y_points = ranges * np.sin(angles)

        # For simplicity, assume the scan matches the robot's current pose
        # In real SLAM, this would involve scan matching
        last_scan = self.current_pose  # For demonstration, we'll use current pose as the last scan
        delta_pose = self.scan_match(last_scan, (x_points, y_points))

        # Update robot pose based on scan matching result
        self.current_pose = self.current_pose + delta_pose
        self.pose_graph.append(self.current_pose)

        # Add edge to the graph representing the transformation
        self.edges.append({'from': len(self.pose_graph) - 2, 'to': len(self.pose_graph) - 1, 'transform': delta_pose})

        # Optionally, you can detect loop closures and add edges to the graph
        loop_closure_idx = self.detect_loop_closure(self.pose_graph, self.current_pose)
        if loop_closure_idx is not None:
            self.edges.append({'from': len(self.pose_graph) - 1, 'to': loop_closure_idx, 'transform': self.current_pose - self.pose_graph[loop_closure_idx]})
        
        # Optimize graph and update map
        optimized_poses = self.optimize_graph(self.pose_graph, self.edges)
        self.generate_map(optimized_poses)

    def odometry_callback(self, msg: Odometry):
        # Extract the position and orientation from the odometry message
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        
        # Convert orientation from quaternion to euler angles
        _, _, yaw = self.quaternion_to_euler(orientation)

        # Update the odometry pose (you can incorporate this into your graph constraints)
        self.odom_pose = np.array([position.x, position.y, yaw])
        
        # Update the robot's current pose using filtered odometry data
        if self.odom_pose is not None:
            self.current_pose = self.odom_pose
            self.pose_graph.append(self.current_pose)
            self.edges.append({'from': len(self.pose_graph) - 2, 'to': len(self.pose_graph) - 1, 'transform': self.odom_pose - self.pose_graph[-2]})

        # After incorporating odometry, run graph optimization and generate the map
        optimized_poses = self.optimize_graph(self.pose_graph, self.edges)
        self.generate_map(optimized_poses)

    def scan_match(self, last_pose, current_scan):
        # Implement scan matching algorithm (simple approach for now)
        # You can use libraries like Open3D, PCL, or ICP for a better solution
        return np.array([0.1, 0.0, 0.05])  # Example delta_pose [dx, dy, dtheta]

    def detect_loop_closure(self, pose_graph, current_pose):
        # Simple loop closure detection (if the robot is close to a previous pose)
        for idx, past_pose in enumerate(pose_graph):
            if np.linalg.norm(past_pose[:2] - current_pose[:2]) < 1.0:
                return idx
        return None

    def optimize_graph(self, pose_graph, edges):
        # You can optimize the graph using g2o, Ceres Solver, or iSAM
        # For simplicity, let's assume we're just returning the current graph for now
        return pose_graph

    def generate_map(self, optimized_poses):
        # Create a map visualization based on optimized poses
        trajectory = np.array(optimized_poses)
        plt.plot(trajectory[:, 0], trajectory[:, 1], label="Optimized Path")
        plt.title("Optimized Robot Trajectory")
        plt.xlabel("X position (m)")
        plt.ylabel("Y position (m)")
        plt.show()

    def quaternion_to_euler(self, quaternion):
        # Convert quaternion to euler angles (roll, pitch, yaw)
        import math
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        # Roll, pitch, yaw (using Tait-bryan angles)
        roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        pitch = math.asin(2.0 * (w * y - z * x))
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

        return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args)
    slam_node = GraphBasedSLAMNode()
    
    # Spin the node to keep processing data
    rclpy.spin(slam_node)

    # Shutdown
    slam_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
