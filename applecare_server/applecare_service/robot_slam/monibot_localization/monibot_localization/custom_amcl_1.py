import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from std_msgs.msg import Header
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import numpy as np
import struct
import random


class SimpleAMCL(Node):
    def __init__(self):
        super().__init__('custom_amcl_node')

        # Parameters
        self.declare_parameter('num_particles', 100)
        self.declare_parameter('motion_noise', [0.05, 0.05])  # [linear, angular]
        self.declare_parameter('sensor_noise', 0.2)
        self.declare_parameter('resample_threshold', 0.5)  # Threshold for resampling

        self.num_particles = self.get_parameter('num_particles').get_parameter_value().integer_value
        self.motion_noise = self.get_parameter('motion_noise').get_parameter_value().double_array_value
        self.sensor_noise = self.get_parameter('sensor_noise').get_parameter_value().double_value
        self.resample_threshold = self.get_parameter('resample_threshold').get_parameter_value().double_value

        # Subscriptions and Publishers
        laser_qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        odom_qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, laser_qos_profile)
        self.odom_sub = self.create_subscription(Odometry, '/odom_filter', self.odom_callback, odom_qos_profile)
        self.initial_pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.initial_pose_callback, 10)

        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/amcl_pose', 10)
        self.particlecloud_pub = self.create_publisher(PointCloud2, '/amcl_particlecloud', 10)

        # Internal state
        self.map = None
        self.particles = []
        self.robot_pose = PoseWithCovarianceStamped()

    def map_callback(self, msg):
        self.map = msg

    def initial_pose_callback(self, msg):
        self.robot_pose = msg
        self.particles = [
            {
                'x': msg.pose.pose.position.x + random.uniform(-0.5, 0.5),
                'y': msg.pose.pose.position.y + random.uniform(-0.5, 0.5),
                'yaw': 0.0,
                'weight': 1.0 / self.num_particles
            }
            for _ in range(self.num_particles)
        ]
        self.get_logger().info("Initial pose received and particles initialized.")

    def odom_callback(self, msg):
        if not self.particles:
            return
        for particle in self.particles:
            # Apply motion model with Gaussian noise
            particle['x'] += msg.twist.twist.linear.x * 0.1 + random.gauss(0, self.motion_noise[0])
            particle['y'] += msg.twist.twist.linear.y * 0.1 + random.gauss(0, self.motion_noise[0])
            particle['yaw'] += msg.twist.twist.angular.z * 0.1 + random.gauss(0, self.motion_noise[1])

    def laser_callback(self, scan_msg):
        if self.map is None:
            self.get_logger().warn("No map received yet!")
            return

        # Process particles and get the estimated pose
        estimated_pose = self.process_particles(scan_msg)

        # Publish the estimated pose
        self.robot_pose.header.stamp = self.get_clock().now().to_msg()
        self.robot_pose.header.frame_id = 'map'
        self.robot_pose.pose.pose.position.x = estimated_pose['x']
        self.robot_pose.pose.pose.position.y = estimated_pose['y']
        self.robot_pose.pose.pose.orientation.z = estimated_pose['orientation_z']
        self.robot_pose.pose.pose.orientation.w = estimated_pose['orientation_w']
        self.pose_pub.publish(self.robot_pose)

        # Publish particle cloud
        self.publish_particle_cloud()

    def process_particles(self, scan_msg):
        # Update particle weights based on laser scan
        total_weight = 0.0
        for particle in self.particles:
            particle['weight'] = self.compute_weight(particle, scan_msg)
            total_weight += particle['weight']

        # Normalize weights
        if total_weight > 0:
            for particle in self.particles:
                particle['weight'] /= total_weight
        else:
            self.get_logger().warn("Total weight is zero, resetting particles.")
            self.reset_particles()

        # Resample particles if effective particle count is low
        effective_particles = 1.0 / sum(p['weight'] ** 2 for p in self.particles)
        if effective_particles / self.num_particles < self.resample_threshold:
            self.resample_particles()

        # Compute weighted average pose
        x = sum(p['x'] * p['weight'] for p in self.particles)
        y = sum(p['y'] * p['weight'] for p in self.particles)
        yaw = sum(p['yaw'] * p['weight'] for p in self.particles)

        return {
            'x': x,
            'y': y,
            'orientation_z': np.sin(yaw / 2),
            'orientation_w': np.cos(yaw / 2),
        }

    def compute_weight(self, particle, scan_msg):
        """
        Compute the weight of a particle based on the laser scan and map.

        Args:
            particle (dict): The particle to evaluate with fields {'x', 'y', 'yaw', 'weight'}.
            scan_msg (LaserScan): The laser scan data.

        Returns:
            float: The weight of the particle.
        """
        if self.map is None:
            return 0.0  # If no map is available, assign zero weight

        # Convert particle pose to map grid coordinates
        map_resolution = self.map.info.resolution
        map_origin_x = self.map.info.origin.position.x
        map_origin_y = self.map.info.origin.position.y

        particle_x_grid = int((particle['x'] - map_origin_x) / map_resolution)
        particle_y_grid = int((particle['y'] - map_origin_y) / map_resolution)

        # Check if the particle is out of map bounds
        if (particle_x_grid < 0 or particle_x_grid >= self.map.info.width or
                particle_y_grid < 0 or particle_y_grid >= self.map.info.height):
            return 0.0  # Out-of-bounds particles have zero weight

        weight = 0.0
        valid_readings = 0

        # Iterate through laser scan ranges
        for i, distance in enumerate(scan_msg.ranges):
            if distance <= scan_msg.range_min or distance >= scan_msg.range_max:
                continue  # Ignore invalid readings

            # Calculate the laser endpoint in the map frame
            angle = particle['yaw'] + scan_msg.angle_min + i * scan_msg.angle_increment
            endpoint_x = particle['x'] + distance * np.cos(angle)
            endpoint_y = particle['y'] + distance * np.sin(angle)

            # Convert to map grid coordinates
            endpoint_x_grid = int((endpoint_x - map_origin_x) / map_resolution)
            endpoint_y_grid = int((endpoint_y - map_origin_y) / map_resolution)

            # Check if the endpoint is within map bounds
            if (0 <= endpoint_x_grid < self.map.info.width and
                    0 <= endpoint_y_grid < self.map.info.height):
                map_index = endpoint_y_grid * self.map.info.width + endpoint_x_grid
                map_cell = self.map.data[map_index]

                if map_cell == 100:  # Occupied cell
                    weight += 1.0
                elif map_cell == 0:  # Free cell
                    weight += 0.5

                valid_readings += 1

        # Normalize the weight by the number of valid readings
        if valid_readings > 0:
            weight /= valid_readings

        return weight

    def resample_particles(self):
        weights = [p['weight'] for p in self.particles]
        indices = random.choices(range(len(self.particles)), weights, k=self.num_particles)
        self.particles = [self.particles[i] for i in indices]

    def publish_particle_cloud(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]

        points = [[p['x'], p['y'], 0.0] for p in self.particles]

        particlecloud_msg = PointCloud2()
        particlecloud_msg.header = header
        particlecloud_msg.height = 1
        particlecloud_msg.width = len(points)
        particlecloud_msg.fields = fields
        particlecloud_msg.is_bigendian = False
        particlecloud_msg.point_step = 12
        particlecloud_msg.row_step = particlecloud_msg.point_step * len(points)
        particlecloud_msg.data = b''.join(struct.pack('fff', *point) for point in points)
        particlecloud_msg.is_dense = True

        self.particlecloud_pub.publish(particlecloud_msg)

    def reset_particles(self):
        self.particles = [
            {'x': self.robot_pose.pose.pose.position.x + random.uniform(-1, 1),
             'y': self.robot_pose.pose.pose.position.y + random.uniform(-1, 1),
             'yaw': 0.0,
             'weight': 1.0 / self.num_particles}
            for _ in range(self.num_particles)
        ]


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
