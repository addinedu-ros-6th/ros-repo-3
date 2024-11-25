import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import numpy as np
import random
from math import cos, sin, pi, sqrt, exp


class SimpleAMCL(Node):
    def __init__(self):
        super().__init__('simple_amcl_node_1')
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT

        # Parameters
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('laser_scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('num_particles', 100)
        self.declare_parameter('resample_threshold', 0.5)

        self.map_topic = self.get_parameter('map_topic').get_parameter_value().string_value
        self.laser_scan_topic = self.get_parameter('laser_scan_topic').get_parameter_value().string_value
        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.num_particles = self.get_parameter('num_particles').get_parameter_value().integer_value
        self.resample_threshold = self.get_parameter('resample_threshold').get_parameter_value().double_value

        # Subscriptions
        self.map_sub = self.create_subscription(OccupancyGrid, self.map_topic, self.map_callback, 10)
        self.laser_scan_sub = self.create_subscription(LaserScan, self.laser_scan_topic, self.laser_callback, qos_profile)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        self.initial_pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/initial_pose', self.initial_pose_callback, 10)

        # Publisher
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/amcl_pose', 10)

        # Internal state
        self.map = None
        self.particles = []  # Particle filter
        self.weights = []  # Particle weights
        self.robot_pose = PoseWithCovarianceStamped()

    def map_callback(self, msg):
        self.get_logger().info("Map received")
        self.map = msg

    def laser_callback(self, scan_msg):
        if self.map is None:
            self.get_logger().warn("No map received yet!")
            return

        if not self.particles:
            self.get_logger().warn("No particles initialized yet!")
            return

        # Update particle weights based on sensor data
        self.update_weights(scan_msg)

        # Resample particles if the effective number drops below the threshold
        if self.effective_particle_count() < self.num_particles * self.resample_threshold:
            self.resample_particles()

        # Estimate the pose from particles
        self.estimate_pose()

    def odom_callback(self, msg):
        if not self.particles:
            return

        # Update particle positions based on odometry
        self.motion_update(msg)

    def initial_pose_callback(self, msg):
        self.get_logger().info("Initial pose received")
        self.initialize_particles(msg.pose.pose.position.x, msg.pose.pose.position.y, 0.0)  # yaw = 0.0 as a placeholder

    def initialize_particles(self, x, y, yaw):
        self.particles = [
            {
                'x': x + random.uniform(-1, 1),
                'y': y + random.uniform(-1, 1),
                'yaw': yaw + random.uniform(-pi / 8, pi / 8),
                'weight': 1.0 / self.num_particles
            }
            for _ in range(self.num_particles)
        ]
        self.weights = [1.0 / self.num_particles] * self.num_particles
        self.get_logger().info(f"Particles initialized at ({x}, {y}, {yaw})")

    def motion_update(self, odom_msg):
        for particle in self.particles:
            dx = odom_msg.twist.twist.linear.x + random.gauss(0, 0.1)
            dy = odom_msg.twist.twist.linear.y + random.gauss(0, 0.1)
            dyaw = odom_msg.twist.twist.angular.z + random.gauss(0, 0.01)

            particle['x'] += dx * cos(particle['yaw']) - dy * sin(particle['yaw'])
            particle['y'] += dx * sin(particle['yaw']) + dy * cos(particle['yaw'])
            particle['yaw'] += dyaw

    def update_weights(self, scan_msg):
        for i, particle in enumerate(self.particles):
            self.weights[i] = self.sensor_model(particle, scan_msg)
        total_weight = sum(self.weights)
        if total_weight > 0:
            self.weights = [w / total_weight for w in self.weights]
        else:
            self.get_logger().warn("All particle weights are zero!")

    def sensor_model(self, particle, scan_msg):
        """
        Simplified likelihood model comparing scan data to map.
        """
        weight = 1.0
        for i, r in enumerate(scan_msg.ranges):
            if r == float('inf') or r == 0.0:
                continue
            angle = scan_msg.angle_min + i * scan_msg.angle_increment + particle['yaw']
            x_hit = particle['x'] + r * cos(angle)
            y_hit = particle['y'] + r * sin(angle)
            map_x = int((x_hit - self.map.info.origin.position.x) / self.map.info.resolution)
            map_y = int((y_hit - self.map.info.origin.position.y) / self.map.info.resolution)
            if 0 <= map_x < self.map.info.width and 0 <= map_y < self.map.info.height:
                map_index = map_y * self.map.info.width + map_x
                if self.map.data[map_index] == 100:
                    weight *= 0.9  # Penalize hits in occupied cells
                elif self.map.data[map_index] == 0:
                    weight *= 1.1  # Reward hits in free cells
        return weight

    def resample_particles(self):
        cumulative_weights = np.cumsum(self.weights)
        new_particles = []
        for _ in range(self.num_particles):
            random_weight = random.uniform(0, 1)
            index = np.searchsorted(cumulative_weights, random_weight)
            new_particles.append(self.particles[index].copy())
        self.particles = new_particles
        self.get_logger().info("Particles resampled.")

    def effective_particle_count(self):
        return 1.0 / sum(w**2 for w in self.weights)

    def estimate_pose(self):
        x = sum(p['x'] * w for p, w in zip(self.particles, self.weights))
        y = sum(p['y'] * w for p, w in zip(self.particles, self.weights))
        yaw = sum(p['yaw'] * w for p, w in zip(self.particles, self.weights))

        # Publish estimated pose
        self.robot_pose.header.stamp = self.get_clock().now().to_msg()
        self.robot_pose.header.frame_id = 'map'
        self.robot_pose.pose.pose.position.x = x
        self.robot_pose.pose.pose.position.y = y
        self.robot_pose.pose.pose.orientation.z = sin(yaw / 2)
        self.robot_pose.pose.pose.orientation.w = cos(yaw / 2)
        self.pose_pub.publish(self.robot_pose)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleAMCL()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
