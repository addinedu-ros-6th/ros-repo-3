#!/usr/bin/env python3
# ekf_filter.py

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Quaternion, Twist
from sensor_msgs.msg import Imu
import tf_transformations
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from tf2_ros import TransformBroadcaster

class EKFSLAM(Node):
    def __init__(self):
        super().__init__('ekf_odom_filter')

        # Load parameters
        self.odom_topic = self.declare_parameter("odom_topic", "/base_controller/odom").get_parameter_value().string_value
        self.imu_topic = self.declare_parameter("imu_topic", "/imu").get_parameter_value().string_value
        self.odom_filtered_topic = self.declare_parameter("odom_filtered_topic", "/odom_filter").get_parameter_value().string_value
        self.update_frequency = self.declare_parameter("update_frequency", 5.0).get_parameter_value().double_value
        self.alpha = self.declare_parameter("alpha", 0.5).get_parameter_value().double_value  # Smoothing factor
        self.odom_position_noise = self.declare_parameter("odom_position_noise", 0.1).get_parameter_value().double_value
        self.imu_yaw_noise = self.declare_parameter("imu_yaw_noise", 0.01).get_parameter_value().double_value

        # Initialize state and covariance
        self.state = np.zeros((3, 1))  # State vector [x, y, theta]
        self.covariance = np.eye(3)  # Initial covariance matrix
        self.prev_time = None
        self.filtered_linear_velocity = 0.0
        self.filtered_angular_velocity = 0.0
        self.imu_yaw = 0.0

        # Flags and message storage
        self.latest_odom_msg = None
        self.latest_imu_msg = None

        # Setup subscriptions and publisher
        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        odom_qos_profile = QoSProfile(depth=10, reliability = QoSReliabilityPolicy.RELIABLE)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, qos_profile)
        self.imu_sub = self.create_subscription(Imu, self.imu_topic, self.imu_callback, qos_profile)
        self.odom_publisher = self.create_publisher(Odometry, self.odom_filtered_topic, odom_qos_profile)

        # Timer for periodic EKF updates
        self.timer = self.create_timer(1.0 / self.update_frequency, self.update_ekf)

        self.get_logger().info("EKFSLAM node initialized with parameters.")

    def update_ekf(self):
        """Main EKF update loop."""
        # Check for the presence of odometry and IMU messages
        if self.latest_odom_msg is None or self.latest_imu_msg is None:
            return
        
        current_time = self.latest_odom_msg.header.stamp.sec + self.latest_odom_msg.header.stamp.nanosec * 1e-9
        if self.prev_time is None:
            self.prev_time = current_time
            return

        dt = current_time - self.prev_time

        # Prepare control input for the prediction step
        control_input = np.array([[self.filtered_linear_velocity], [self.filtered_angular_velocity], [dt]])
        self.prediction_step(control_input)

        # Run the odometry correction step
        self.odometry_correction_step(self.latest_odom_msg.pose.pose)

        self.prev_time = current_time
        self.publish_filtered_odom() # Publish the filtered odometry message


    def odom_callback(self, msg):
        """Callback for odometry messages."""
        self.latest_odom_msg = msg
        dt = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.prev_time:
            dt -= self.prev_time
        linear_velocity = msg.twist.twist.linear.x
        angular_velocity = msg.twist.twist.angular.z

        # Apply low-pass filter to smooth velocities
        self.filtered_linear_velocity = (self.alpha * linear_velocity 
                                         + (1 - self.alpha) * self.filtered_linear_velocity)
        self.filtered_angular_velocity = (self.alpha * angular_velocity 
                                          + (1 - self.alpha) * self.filtered_angular_velocity)


    def imu_callback(self, imu_msg):
        """Callback for IMU messages."""
        self.latest_imu_msg = imu_msg
        quaternion = [imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w]
        _, _, self.imu_yaw = tf_transformations.euler_from_quaternion(quaternion)


    def prediction_step(self, control_input):
        """EKF Prediction Step."""
        v, w, dt = control_input.flatten()
        # v = control_input[0, 0]  # Linear velocity
        # w = control_input[1, 0]  # Angular velocity
        # dt = control_input[2, 0]  # Time step

        theta = self.state[2, 0]

        # Propagate state using motion model
        self.state[0, 0] += v * dt * np.cos(theta)
        self.state[1, 0] += v * dt * np.sin(theta)
        self.state[2, 0] += w * dt
        self.state[2, 0] = (self.state[2, 0] + np.pi) % (2 * np.pi) - np.pi

        # Update covariance
        F_x = np.array([[1, 0, -v * dt * np.sin(theta)],
                        [0, 1, v * dt * np.cos(theta)],
                        [0, 0, 1]])
        G = np.array([[np.cos(theta), 0],
                      [np.sin(theta), 0],
                      [0, dt]])
        Q = np.eye(2) * 0.1
        self.covariance = F_x @ self.covariance @ F_x.T + G @ Q @ G.T


    def odometry_correction_step(self, odom_pose):
        """EKF Correction Step with odometry and IMU data."""
        # Extract position from odometry
        odom_x, odom_y = odom_pose.position.x, odom_pose.position.y
        # quaternion = [odom_pose.orientation.x, odom_pose.orientation.y,
        #               odom_pose.orientation.z, odom_pose.orientation.w]
        # _, _, odom_theta = tf_transformations.euler_from_quaternion(quaternion)

        """Correct x, y with odometry"""
        # Measurement model H (2, 3) for x, y, theta with theta handled by IMU
        H = np.array([[1, 0, 0], # x measurement
                      [0, 1, 0]])  # y measurement

        # Measurement noise covariance R for odometry
        R_odom = np.eye(2) * self.odom_position_noise

        # Measurement vector from odometry for x, y
        measurement = np.array([[odom_x], [odom_y]])

        # Innovation for x, y
        innovation = measurement - self.state[:2]

        # Innovation covariance for x, y
        #   (2,3)     @    (3, 3)     @ (3, 2)
        S = H @ self.covariance @ H.T + R_odom # Innovation covariance (2 x 2)

        # Kalman gain for x, y
        K = self.covariance @ H.T @ np.linalg.inv(S) # Kalman gain (3 x 2)

        # Update x, y position only with odometry
        self.state += K @ innovation
        self.covariance = (np.eye(self.covariance.shape[0]) - K @ H) @ self.covariance

        """Correct theta with IMU"""
        # Correction for yaw using IMU with higher weight (lower covariance)
        H_imu = np.array([[0, 0, 1]])
        R_imu = np.array([[self.imu_yaw_noise]])  # Higher trust in IMU data for yaw
        imu_measurement = np.array([[self.imu_yaw]])

        # Innovation and Kalman gain for yaw
        innovation_yaw = imu_measurement - self.state[2]
        S_yaw = H_imu @ self.covariance @ H_imu.T + R_imu
        K_yaw = self.covariance @ H_imu.T @ np.linalg.inv(S_yaw)

        # Update yaw with IMU data
        self.state += K_yaw @ innovation_yaw
        self.covariance = (np.eye(self.covariance.shape[0]) - K_yaw @ H_imu) @ self.covariance

    def publish_filtered_odom(self):
        # Create the Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'

        # Set position and orientation from EKF state
        odom_msg.pose.pose.position.x = self.state[0, 0]
        odom_msg.pose.pose.position.y = self.state[1, 0]
        
        # Convert yaw angle to Quaternion for orientation
        #quaternion = self.get_quaternion_from_yaw(self.state[2, 0])
        quaternion = tf_transformations.quaternion_from_euler(0, 0, self.state[2, 0])
        
        odom_msg.pose.pose.orientation = Quaternion(
            x=quaternion[0],
            y=quaternion[1],
            z=quaternion[2],
            w=quaternion[3]
        )

        # Set linear and angular velocity
        odom_msg.twist.twist.linear.x = self.filtered_linear_velocity
        odom_msg.twist.twist.angular.z = self.filtered_angular_velocity

        # Publish the odometry message
        self.odom_publisher.publish(odom_msg)
        # print("filtered odometry :", odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = EKFSLAM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
