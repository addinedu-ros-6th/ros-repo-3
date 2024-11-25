import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import numpy as np
import random


class SimpleAMCL(Node):
    def __init__(self):
        super().__init__('simple_amcl_node_2')

        # Parameters
        self.declare_parameter('num_particles', 100)
        self.declare_parameter('motion_noise', [0.05, 0.05])  # [linear, angular]
        self.declare_parameter('sensor_noise', 0.2)

        self.num_particles = self.get_parameter('num_particles').get_parameter_value().integer_value
        self.motion_noise = self.get_parameter('motion_noise').get_parameter_value().double_array_value
        self.sensor_noise = self.get_parameter('sensor_noise').get_parameter_value().double_value

        # Subscriptions and Publishers
        laser_qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        odom_qos_profile = QoSProfile(depth=10, reliability = QoSReliabilityPolicy.RELIABLE)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, laser_qos_profile)
        self.odom_sub = self.create_subscription(Odometry, '/odom_filter', self.odom_callback, odom_qos_profile)
        self.initial_pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.initial_pose_callback, 10)
        
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/amcl_pose', 10)

        # Internal state
        self.map = None
        self.particles = []
        self.robot_pose = PoseWithCovarianceStamped()

    def map_callback(self, msg):
        self.map = msg
        self.get_logger().info("Map received.")

    def initial_pose_callback(self, msg):
        self.robot_pose = msg
        self.particles = [
            {'x': msg.pose.pose.position.x + random.uniform(-1, 1),
             'y': msg.pose.pose.position.y + random.uniform(-1, 1),
             'yaw': 0.0,
             'weight': 1.0 / self.num_particles}
            for _ in range(self.num_particles)
        ]
        self.get_logger().info("Initial pose received and particles initialized.")

    def odom_callback(self, msg):
        if not self.particles:
            return
        for particle in self.particles:
            particle['x'] += msg.twist.twist.linear.x + random.gauss(0, self.motion_noise[0])
            particle['y'] += msg.twist.twist.linear.y + random.gauss(0, self.motion_noise[0])
            particle['yaw'] += msg.twist.twist.angular.z + random.gauss(0, self.motion_noise[1])

    def laser_callback(self, scan_msg):
        if self.map is None:
            self.get_logger().warn("No map received yet!")
            return

        self.get_logger().info("Processing laser scan...")
        
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

        self.get_logger().info("Published estimated pose.")

    def process_particles(self, scan_msg):
        """
        Process particles: Update weights based on scan, normalize, and compute pose.
        
        Args:
            scan_msg (LaserScan): Laser scan data to update particle weights.
        
        Returns:
            dict: Estimated pose {'x': float, 'y': float, 'orientation_z': float, 'orientation_w': float}.
        """
        # Update particle weights based on laser scan matching
        total_weight = 0.0
        for particle in self.particles:
            particle['weight'] *= self.compute_weight(particle, scan_msg)
            total_weight += particle['weight']
        
        # Normalize particle weights
        if total_weight > 0:
            for particle in self.particles:
                particle['weight'] /= total_weight
        
        # Compute weighted average pose
        x = float(sum(p['x'] * p['weight'] for p in self.particles))
        y = float(sum(p['y'] * p['weight'] for p in self.particles))
        orientation_z = float(sum(p['yaw'] * p['weight'] for p in self.particles))
        
        return {
            'x': x,
            'y': y,
            'orientation_z': orientation_z,
            'orientation_w': 1.0  # Assume no tilt
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

    # def laser_callback(self, msg):
    #     if not self.particles or self.map is None:
    #         return

    #     for particle in self.particles:
    #         particle['weight'] = self.calculate_weight(particle, msg)

    #     total_weight = sum(p['weight'] for p in self.particles)
    #     if total_weight > 0:
    #         for particle in self.particles:
    #             particle['weight'] /= total_weight
    #         self.resample_particles()
    #         self.publish_pose()

    # def calculate_weight(self, particle, laser_scan):
    #     weight = 1.0
    #     for i, r in enumerate(laser_scan.ranges):
    #         if r < laser_scan.range_min or r > laser_scan.range_max:
    #             continue

    #         angle = laser_scan.angle_min + i * laser_scan.angle_increment
    #         x = particle['x'] + r * np.cos(particle['yaw'] + angle)
    #         y = particle['y'] + r * np.sin(particle['yaw'] + angle)

    #         grid_x = int((x - self.map.info.origin.position.x) / self.map.info.resolution)
    #         grid_y = int((y - self.map.info.origin.position.y) / self.map.info.resolution)

    #         if 0 <= grid_x < self.map.info.width and 0 <= grid_y < self.map.info.height:
    #             map_index = grid_y * self.map.info.width + grid_x
    #             if self.map.data[map_index] == 100:  # Obstacle
    #                 weight *= self.sensor_noise
    #             elif self.map.data[map_index] == 0:  # Free space
    #                 weight *= (1 - self.sensor_noise)
    #     return weight

    # def resample_particles(self):
    #     weights = [p['weight'] for p in self.particles]
    #     indices = random.choices(range(len(self.particles)), weights, k=self.num_particles)
    #     self.particles = [self.particles[i] for i in indices]

    # def publish_pose(self):
    #     x = sum(p['x'] * p['weight'] for p in self.particles)
    #     y = sum(p['y'] * p['weight'] for p in self.particles)
    #     yaw = sum(p['yaw'] * p['weight'] for p in self.particles)

    #     self.robot_pose.header.stamp = self.get_clock().now().to_msg()
    #     self.robot_pose.header.frame_id = 'map'
    #     self.robot_pose.pose.pose.position.x = x
    #     self.robot_pose.pose.pose.position.y = y
    #     self.robot_pose.pose.pose.orientation.z = np.sin(yaw / 2)
    #     self.robot_pose.pose.pose.orientation.w = np.cos(yaw / 2)
    #     self.pose_pub.publish(self.robot_pose)
    #     self.get_logger().info("Published estimated pose.")


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
