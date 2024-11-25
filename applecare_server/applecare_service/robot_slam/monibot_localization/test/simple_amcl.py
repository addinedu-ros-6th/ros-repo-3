import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class SimpleAMCL(Node):
    def __init__(self):
        super().__init__('simple_amcl_node')
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT

        # Declare parameters
        # self.declare_parameter('initial_pose_x', 1.0)
        # self.declare_parameter('initial_pose_y', 2.0)
        # self.declare_parameter('initial_pose_yaw', 0.785)
        
        # Declare additional parameters
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('laser_scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/odom_filter')

        # Parameters
        # self.initial_pose_x = self.get_parameter('initial_pose_x').get_parameter_value().double_value
        # self.initial_pose_y = self.get_parameter('initial_pose_y').get_parameter_value().double_value
        # self.initial_pose_yaw = self.get_parameter('initial_pose_yaw').get_parameter_value().double_value

        self.map_topic = self.get_parameter('map_topic').get_parameter_value().string_value
        self.laser_scan_topic = self.get_parameter('laser_scan_topic').get_parameter_value().string_value
        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value

        # Subscriptions
        self.map_sub = self.create_subscription(OccupancyGrid, self.map_topic, self.map_callback, 10)
        self.laser_scan_sub = self.create_subscription(LaserScan, self.laser_scan_topic, self.laser_callback, qos_profile)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        self.initial_pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/initial_pose', self.initial_pose_callback, 10)

        # Publishers
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/amcl_pose', 10)

        # Internal state
        self.map = None
        self.particles = []  # Placeholder for particle filter
        self.robot_pose = PoseWithCovarianceStamped()

    def map_callback(self, msg):
        self.get_logger().info("Map received")
        self.map = msg  # Store the map for localization

    def laser_callback(self, msg):
        if self.map is None:
            self.get_logger().warn("No map received yet!")
            return

        # Placeholder: Process laser scan, update particle weights
        self.get_logger().info("Processing laser scan...")

        # Example: Call your scan matching function (implement this)
        estimated_pose = self.scan_match(msg)

        # Publish the estimated pose
        self.robot_pose.header.stamp = self.get_clock().now().to_msg()
        self.robot_pose.header.frame_id = 'map'
        self.robot_pose.pose.pose.position.x = estimated_pose['x']
        self.robot_pose.pose.pose.position.y = estimated_pose['y']
        self.robot_pose.pose.pose.orientation.w = 1.0  # Update with actual yaw
        self.pose_pub.publish(self.robot_pose)

    def odom_callback(self, msg):
        self.get_logger().info("Processing odometry data...")

        # Update particles using odometry (simplified)
        for particle in self.particles:
            particle['x'] += msg.twist.twist.linear.x * 0.1  # Example update
            particle['yaw'] += msg.twist.twist.angular.z * 0.1  # Example update


    def scan_match(self, scan_msg):
        """
        Match the laser scan data with the map and estimate the robot's pose.

        Args:
            scan_msg (LaserScan): Current laser scan data.

        Returns:
            dict: Estimated pose {'x': float, 'y': float, 'yaw': float}.
        """
        # Placeholder: Replace with actual scan-matching logic
        # For example, iterate through particles and compute the best match
        best_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        return best_pose

    def initial_pose_callback(self, msg):
        self.get_logger().info("Initial pose received")
        self.robot_pose = msg  # Set initial pose

        # Initialize particle filter based on initial pose (simplified)
        self.particles = [
            {
                'x': msg.pose.pose.position.x,
                'y': msg.pose.pose.position.y,
                'yaw': 0.0,  # Example: extract from orientation
                'weight': 1.0
            }
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
