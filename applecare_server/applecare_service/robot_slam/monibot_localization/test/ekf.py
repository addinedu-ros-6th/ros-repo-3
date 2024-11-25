#ekf_odom.py
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


class EKFSLAM(Node):
    def __init__(self):
        super().__init__('ekf_filter')
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT

        # https://docs.ros.org/en/humble/Tutorials/Demos/Content-Filtering-Subscription.html
        # create_subscription vs subscriber
        self.odom_sub = self.create_subscription(Odometry, '/base_controller/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile)
        self.imu_sub = self.create_subscription(String, '/imu_data', self.imu_callback, qos_profile)
        self.odom_publisher = self.create_publisher(Odometry, 'odom_filtered', 10)

        # Initialize EKF state, covariance, and parameters
        self.state = np.zeros((3, 1))  # State vector [x, y, theta]
        self.covariance = np.eye(3)  # Initial covariance matrix
        self.last_imu_yaw = 0.0  # To store last IMU yaw
        self.prev_time = None
        self.update_frequency = 0.2  # 3 Hz
        self.update_period = 1.0 / self.update_frequency  # Convert Hz to seconds
        self.timer = self.create_timer(self.update_period, self.update_ekf)
        self.alpha = 0.5  # Smoothing factor, can be tuned
        self.filtered_linear_velocity = 0.0
        self.filtered_angular_velocity = 0.0

        #Variables to store the latest data
        self.latest_odom_msg = None
        self.latest_scan_msg = None
        self.latest_imu_msg = None

    def update_ekf(self):
        # Ensure we have received at least one message before processing
        if self.latest_odom_msg is None or self.latest_scan_msg is None:
            return

        current_time = self.latest_odom_msg.header.stamp.sec + self.latest_odom_msg.header.stamp.nanosec * 1e-9
        if self.prev_time is not None:
            dt = current_time - self.prev_time
            linear_velocity = self.latest_odom_msg.twist.twist.linear.x
            angular_velocity = self.latest_odom_msg.twist.twist.angular.z
            control_input = np.array([[linear_velocity], [angular_velocity], [dt]])

            # Run EKF prediction and update steps
            self.prediction_step(control_input)
            
            # Run update with odometry correction
            odom_pose = self.latest_odom_msg.pose.pose
            self.odometry_correction_step(odom_pose)
        
        self.prev_time = current_time
        self.publish_filtered_odom()
        
    
    def odometry_correction_step(self, odom_pose):
        # Extract position and orientation from odometry
        odom_x = odom_pose.position.x
        odom_y = odom_pose.position.y
        quaternion = [odom_pose.orientation.x, odom_pose.orientation.y,
                      odom_pose.orientation.z, odom_pose.orientation.w]
        _, _, odom_theta = tf_transformations.euler_from_quaternion(quaternion)

        # Measurement vector from odometry
        measurement = np.array([[odom_x], [odom_y], [odom_theta]])

        # Calculate innovation (difference between measured and predicted state)
        innovation = measurement - self.state

        # Measurement model H (Jacobian for observation)
        H = np.eye(3)  # Direct mapping for simplicity
        
        # Measurement noise covariance R
        R = np.eye(3) * 0.05  # Can be tuned based on odometry accuracy

        # Innovation covariance
        S = H @ self.covariance @ H.T + R

        # Kalman gain
        K = self.covariance @ H.T @ np.linalg.inv(S)

        # State update with correction
        self.state += K @ innovation
        self.covariance = (np.eye(self.covariance.shape[0]) - K @ H) @ self.covariance

    
    def publish_filtered_odom(self):
        # Create the Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # Set position and orientation from EKF state
        odom_msg.pose.pose.position.x = self.state[0, 0]
        odom_msg.pose.pose.position.y = self.state[1, 0]
        
        # Convert yaw angle to Quaternion for orientation
        quaternion = self.get_quaternion_from_yaw(self.state[2, 0])
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


    def get_quaternion_from_yaw(self, yaw):
        """Helper function to convert yaw to quaternion"""
        return tf_transformations.quaternion_from_euler(0, 0, yaw)
    
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
            #print("before EKF: ", control_input)

        else:
            self.get_logger().info('This is the first message, dt cannot be calculated.')

        # Update previous time for next callback
        self.prev_time = current_time
        self.latest_odom_msg = msg

    def scan_callback(self, scan_msg):
        # if not scan_msg.header.stamp:
        #     self.get_logger().warn("Received a message without a timestamp. Adding default timestamp.")
        #     scan_msg.header.stamp = rclpy.time.Time().to_msg()  # Assign a default timestamp if missing

        # # Convert scan to observations (e.g., detected landmarks)
        # observations = self.extract_landmarks(scan_msg)
        # self.update_step(observations)
        self.latest_scan_msg = scan_msg


    def imu_callback(self, imu_msg):
        # Update the last IMU yaw to incorporate IMU data into the state
        # self.last_imu_yaw = imu_msg.orientation.z  # Assuming z represents yaw in quaternion     

        # Access the actual string data
        imu_data = imu_msg.data  # Accessing the data attribute of the message

        # Remove the asterisk and newline characters, and split by commas
        imu_values = imu_data.strip("*\r\n").split(",")

        # Convert each string value to a float
        try:
            roll = float(imu_values[0])
            pitch = float(imu_values[1])
            yaw = float(imu_values[2])

            self.last_imu_yaw = yaw
            self.latest_imu_msg = imu_msg
            #self.get_logger().info(f"Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}")
        
        except (ValueError, IndexError) as e:
            self.get_logger().info(f"Error parsing IMU data: {e}")


    def prediction_step(self, control_input):
        v = control_input[0, 0]  # Linear velocity
        w = control_input[1, 0]  # Angular velocity
        dt = control_input[2, 0]  # Time step
        theta = self.state[2, 0] + self.last_imu_yaw # Current orientation
        
        # Update state with motion model (propagate state)
        self.state[0, 0] += v * dt * np.cos(theta)
        self.state[1, 0] += v * dt * np.sin(theta)
        self.state[2, 0] += w * dt  # Update orientation with angular velocity
        self.state[2, 0] = (self.state[2, 0] + np.pi) % (2 * np.pi) - np.pi # Ensure theta is within -pi to pi

        # Update covariance
        F_x = np.array([[1, 0, -v * dt * np.sin(theta)],
                        [0, 1, v * dt * np.cos(theta)],
                        [0, 0, 1]])
        
        G = np.array([[np.cos(theta), 0],
                    [np.sin(theta), 0],
                    [0, dt]])
        
        Q = np.eye(2) * 0.1  # Process noise for control input

        self.covariance = F_x @ self.covariance @ F_x.T + G @ Q @ G.T
        #print("State after prediction:", self.state)
        #print("Covariance after prediction:", self.covariance)


    def update_step(self, observations):
        """
        Update step for EKF with RANSAC to filter out noise and add new landmarks.
        Observations are expected to be in the form [x, y].
        """

        valid_landmarks = self.ransac_filter(observations)

        for obs in valid_landmarks:
            obs_x, obs_y = obs

            # Calculate the expected measurement and innovation covariance
            H = np.zeros((2, self.state.shape[0]))  # Observation model for a single landmark
            H[0, 0] = 1  # Partial derivative of x
            H[1, 1] = 1  # Partial derivative of y

            # Measurement noise covariance R
            R = np.eye(2) * 0.05  # Assuming small measurement noise

            # Expected observation position from the state
            innovation = np.array([[obs_x - self.state[0, 0]],
                                [obs_y - self.state[1, 0]]])

            # Innovation covariance
            S = H @ self.covariance @ H.T + R

            # Kalman gain
            K = self.covariance @ H.T @ np.linalg.inv(S)

            # Update state and covariance
            self.state += K @ innovation
            self.covariance = (np.eye(self.covariance.shape[0]) - K @ H) @ self.covariance

            #print("Updated State:", self.state)

    def add_landmark_to_state(self, x, y):
        """
        Adds a new landmark (x, y) to the state and updates the covariance matrix.
        """
        print(f"Adding new landmark at ({x}, {y})")  # Debugging statement

        # Expand the state vector and add the new landmark
        self.state = np.vstack((self.state, np.array([[x], [y]])))

        # Expand the covariance matrix to include new landmark with large initial uncertainty
        cov_size = self.covariance.shape[0]
        new_cov = np.zeros((cov_size + 2, cov_size + 2))
        new_cov[:cov_size, :cov_size] = self.covariance
        new_cov[cov_size:, cov_size:] = np.eye(2) * 1000  # High initial uncertainty for new landmarks
        self.covariance = new_cov

    def ransac_filter(self, observations, threshold=0.5, max_iterations=100):
        """
        Uses RANSAC to filter out noisy observations, returning a set of valid landmarks.
        """
        best_inliers = []
        for _ in range(max_iterations):
            # Randomly select a sample observation
            sample_idx = np.random.randint(0, len(observations))
            sample = observations[sample_idx]

            # Calculate inliers based on distance threshold
            inliers = []
            for obs in observations:
                if distance.euclidean(sample, obs) < threshold:
                    inliers.append(obs)

            # Update best inliers if current sample has more inliers
            if len(inliers) > len(best_inliers):
                best_inliers = inliers

        # Return the most consistent set of landmarks
        return best_inliers


    def extract_landmarks(self, scan_msg):
        # Basic method to convert laser scan to potential landmark observations
        landmarks = []
        for i, r in enumerate(scan_msg.ranges):
            if r < scan_msg.range_max:  # Filter out invalid readings
                angle = scan_msg.angle_min + i * scan_msg.angle_increment
                x = r * np.cos(angle)
                y = r * np.sin(angle)
                landmarks.append([x, y])
        return landmarks
    

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
