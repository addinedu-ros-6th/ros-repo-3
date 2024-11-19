#!/usr/bin/env python3
# ekf_odom.py

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Quaternion, Twist
from sensor_msgs.msg import LaserScan, Imu
from std_msgs.msg import String
from scipy.spatial import distance
import tf_transformations
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class EKFSLAM(Node):
    def __init__(self):
        super().__init__('ekf_odom_filter')
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT

        # https://docs.ros.org/en/humble/Tutorials/Demos/Content-Filtering-Subscription.html
        # create_subscription vs subscriber
        self.odom_sub = self.create_subscription(Odometry, 'base_controller/odom', self.odom_callback, 10)
        self.imu_sub = self.create_subscription(Imu, 'imu', self.imu_callback, qos_profile)
        self.odom_publisher = self.create_publisher(Odometry, 'odom_filter', 10)
        self.tf_broadcaster = TransformBroadcaster(self)


        # Initialize EKF state and covariance
        self.state = np.zeros((3, 1))  # State vector [x, y, theta]
        self.covariance = np.eye(3)  # Initial covariance matrix
        self.prev_time = None
        self.update_frequency = 0.2  # 3 Hz
        self.update_period = 1.0 / self.update_frequency
        self.timer = self.create_timer(self.update_period, self.update_ekf)

        self.latest_odom_msg = None
        self.latest_scan_msg = None
        self.latest_imu_msg = None
        self.alpha = 0.5  # Smoothing factor for velocity filtering
        self.filtered_linear_velocity = 0.0
        self.filtered_angular_velocity = 0.0
        self.imu_yaw = 0.0  # Latest yaw angle from IMU

    
    def update_ekf(self):
        if self.latest_odom_msg is None or self.latest_scan_msg is None or self.latest_imu_msg is None:
            return

        current_time = self.latest_odom_msg.header.stamp.sec + self.latest_odom_msg.header.stamp.nanosec * 1e-9
        if self.prev_time is not None:
            dt = current_time - self.prev_time
            linear_velocity = self.filtered_linear_velocity
            angular_velocity = self.filtered_angular_velocity
            control_input = np.array([[linear_velocity], [angular_velocity], [dt]])
            self.prediction_step(control_input)

            # Run update with odometry correction and IMU correction for yaw
            odom_pose = self.latest_odom_msg.pose.pose
            self.odometry_correction_step(odom_pose)
        
        self.prev_time = current_time
        self.publish_filtered_odom()

    
    def odom_callback(self, msg):
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        if self.prev_time is not None:
            dt = current_time - self.prev_time

            # Assuming msg has the linear and angular velocity, and you can get the time step
            linear_velocity = msg.twist.twist.linear.x #linear.x gives the forward velocity.
            angular_velocity = msg.twist.twist.angular.z #angular.z gives the rotational speed around the z-axis (yaw).

            # control_input = np.array([[linear_velocity], [angular_velocity], [dt]])  # Shape (3, 1)

            # low-pass filter
            self.filtered_linear_velocity = (self.alpha * linear_velocity + 
                                 (1 - self.alpha) * self.filtered_linear_velocity)
            self.filtered_angular_velocity = (self.alpha * angular_velocity + 
                                            (1 - self.alpha) * self.filtered_angular_velocity)

            # Use the filtered values in control_input instead
            control_input = np.array([[self.filtered_linear_velocity], 
                                    [self.filtered_angular_velocity], 
                                    [dt]])

            self.prediction_step(control_input)
            # print("before EKF: ", control_input)

        else:
            self.get_logger().info('This is the first message, dt cannot be calculated.')

        # Update previous time for next callback
        self.prev_time = current_time
        self.latest_odom_msg = msg


    def imu_callback(self, imu_msg):
        # Convert quaternion to yaw (only yaw is used for orientation correction)
        quaternion = [imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w]
        _, _, self.imu_yaw = tf_transformations.euler_from_quaternion(quaternion)
        self.latest_imu_msg = imu_msg


    def prediction_step(self, control_input):
        v = control_input[0, 0]  # Linear velocity
        w = control_input[1, 0]  # Angular velocity
        dt = control_input[2, 0]  # Time step

        # Current orientation
        theta = self.state[2, 0]
        
        # Update state with motion model (propagate state)
        self.state[0, 0] += v * dt * np.cos(theta)
        self.state[1, 0] += v * dt * np.sin(theta)
        self.state[2, 0] += w * dt  # Update orientation with angular velocity

        # Ensure theta is within -pi to pi
        self.state[2, 0] = (self.state[2, 0] + np.pi) % (2 * np.pi) - np.pi

        # Update covariance
        F_x = np.array([[1, 0, -v * dt * np.sin(theta)],
                        [0, 1, v * dt * np.cos(theta)],
                        [0, 0, 1]])
        
        G = np.array([[np.cos(theta), 0],
                    [np.sin(theta), 0],
                    [0, dt]])
        
        Q = np.eye(2) * 0.1  # Process noise for control input

        self.covariance = F_x @ self.covariance @ F_x.T + G @ Q @ G.T
        # print("State after prediction:", self.state)
        #print("Covariance after prediction:", self.covariance)


    def odometry_correction_step(self, odom_pose):
        # Extract position from odometry
        odom_x = odom_pose.position.x
        odom_y = odom_pose.position.y
        quaternion = [odom_pose.orientation.x, odom_pose.orientation.y,
                      odom_pose.orientation.z, odom_pose.orientation.w]
        _, _, odom_theta = tf_transformations.euler_from_quaternion(quaternion)

        # Measurement vector from odometry for x, y
        measurement = np.array([[odom_x], [odom_y]])

        # Measurement model H for x, y, theta with theta handled by IMU
        H = np.array([[1, 0, 0],
                      [0, 1, 0]])

        # Measurement noise covariance R for odometry
        R_odom = np.eye(2) * 0.1

        # Innovation for x, y
        innovation = measurement - self.state[:2]

        # Innovation covariance for x, y
        S = H @ self.covariance[:2, :2] @ H.T + R_odom

        # Kalman gain for x, y
        K = self.covariance[:2, :2] @ H.T @ np.linalg.inv(S)

        # Update x, y position only with odometry
        self.state[:2] += K @ innovation
        self.covariance[:2, :2] = (np.eye(2) - K @ H) @ self.covariance[:2, :2]

        # Correction for yaw using IMU with higher weight (lower covariance)
        imu_measurement = np.array([[self.imu_yaw]])
        H_imu = np.array([[0, 0, 1]])
        R_imu = np.array([[0.01]])  # Higher trust in IMU data for yaw

        # Innovation and Kalman gain for yaw
        innovation_yaw = imu_measurement - self.state[2]
        S_yaw = H_imu @ self.covariance @ H_imu.T + R_imu
        K_yaw = self.covariance @ H_imu.T @ np.linalg.inv(S_yaw)

        # Update yaw with IMU data
        self.state[2] += K_yaw @ innovation_yaw
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
        # odom_msg.twist.twist.linear.x = self.filtered_linear_velocity
        # odom_msg.twist.twist.angular.z = self.filtered_angular_velocity

        # Publish the odometry message
        self.odom_publisher.publish(odom_msg)

        # Broadcast the transform from 'odom' to 'base_footprint'
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_footprint'

        transform.transform.translation.x = self.state[0, 0]
        transform.transform.translation.y = self.state[1, 0]
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = quaternion[0]
        transform.transform.rotation.y = quaternion[1]
        transform.transform.rotation.z = quaternion[2]
        transform.transform.rotation.w = quaternion[3]

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(transform)

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
