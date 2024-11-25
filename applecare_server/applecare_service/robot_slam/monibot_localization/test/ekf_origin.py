#ekf_slam.py
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Imu
from scipy.spatial import distance
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy


class EKFSLAM(Node):
    def __init__(self):
        super().__init__('ekf_slam_node')
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT

        self.odom_sub = self.create_subscription(Odometry, '/base_controller/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, qos_profile)
        # self.imu_sub = self.create_subscription(Imu, 'imu', self.imu_callback, 10)

        # Initialize EKF state and covariance
        self.state = np.zeros((3, 1))  # State vector [x, y, theta]
        self.covariance = np.eye(3)  # Initial covariance matrix
        self.last_imu_yaw = 0.0  # To store last IMU yaw

    def odom_callback(self, msg):

        # # Get control input from odometry (wheel encoder)
        # control_input = np.array([[odom_msg.twist.twist.linear.x],
        #                            [odom_msg.twist.twist.angular.z]])
        # self.prediction_step(control_input)
        
        # --------- option 1
        # Assuming msg has the linear and angular velocity, and you can get the time step
        linear_velocity = msg.twist.twist.linear.x  # Example, replace with your message structure
        angular_velocity = msg.twist.twist.angular.z  # Example, replace with your message structure
        dt = 0.1  # Set this to your actual time step (e.g., time elapsed since the last update)

        control_input = np.array([[linear_velocity], [angular_velocity], [dt]])  # Shape (3, 1)
        self.prediction_step(control_input)

    def scan_callback(self, scan_msg):
        # Convert scan to observations (e.g., detected landmarks)
        observations = self.extract_landmarks(scan_msg)
        self.update_step(observations)

    # def imu_callback(self, imu_msg):
    #     # Update the last IMU yaw to incorporate IMU data into the state
    #     self.last_imu_yaw = imu_msg.orientation.z  # Assuming z represents yaw in quaternion

    def prediction_step(self, control_input):
        # Define Fx based on the current number of state variables
        state_size = self.state.shape[0]
        Fx = np.eye(state_size)

        # Update state with motion model
        dt = control_input[2, 0]  # Time step from control input
        
        # Calculate the current theta by combining odometry and IMU data
        theta = self.state[2, 0] + self.last_imu_yaw

        # Update state based on control inputs and dead reckoning
        # Jacobian of the prediction model
        Fx = np.array([[1, 0, -control_input[0, 0] * dt * np.sin(theta)],
                       [0, 1, control_input[0, 0] * dt * np.cos(theta)],
                       [0, 0, 1]])

        # Control model B
        # B = np.array([[dt * np.cos(theta), 0],
        #               [dt * np.sin(theta), 0],
        #               [0, dt]])

        # Predict state and covariance
        # Assuming self.state is structured as [x, y, theta, ...]
        print("Control input shape:", control_input.shape)  # For debugging
        # self.state += B @ control_input

        # # ----------------- option 1
        # self.state[0:3] += B @ control_input
        # control_input_full = np.array([[control_input[0]], [control_input[1]], [0], [0], [0], [0], [0], [0], [0]])
        # self.state += B @ control_input_full

        # # ----------------- option 2
        print("Before state:", self.state)  # For debugging
        B = self.calculate_control_matrix(control_input)  # Ensure B is correctly computed
        self.state[0:3] += B @ control_input[0:2]  # Update only the position (linear and angular velocity)
        #self.state[0:3] += B @ control_input  # Update only the position (linear and angular velocity)

        # ----------------- option 3
        # control_input_full = np.zeros((9, 1))  # Initialize a (9, 1) array
        # control_input_full[0:2] = control_input[0:2]  # Fill the first two elements
        # # The remaining elements are zero
        # B = self.calculate_control_matrix(control_input)  # Ensure B is correctly computed
        # self.state[0:3] += B @ control_input_full  # Update only the position
        # --------------------------------------

        # self.covariance = Fx @ self.covariance @ Fx.T + np.eye(3) * 0.1  # Process noise
        self.covariance = Fx @ self.covariance @ Fx.T + np.eye(state_size) * 0.1  # Process noise

        print("Fx :", Fx)
        print("Covariance :", self.covariance)

    def update_step(self, observations):
        """
        Update step for EKF with RANSAC to filter out noise and add new landmarks.
        Observations are expected to be in the form [x, y].
        """
        valid_landmarks = self.ransac_filter(observations)  # Filter observations using RANSAC

        for obs in valid_landmarks:
            obs_x, obs_y = obs

            # Calculate distance from each landmark to observation to see if it already exists
            is_new_landmark = True
            for i in range(3, self.state.shape[0], 2):
                # Extract the position of the existing landmark
                landmark_x = self.state[i, 0]
                landmark_y = self.state[i + 1, 0]
                
                # Check if the landmark is close enough to an existing landmark
                if distance.euclidean((obs_x, obs_y), (landmark_x, landmark_y)) < 0.5:  # Threshold distance
                    is_new_landmark = False
                    break

            # If it's a new landmark, add it to the state and update the covariance matrix
            if is_new_landmark:
                self.add_landmark_to_state(obs_x, obs_y)

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
                print("landmark found: ", landmarks)
        return landmarks
    
    def calculate_control_matrix(self, control_input):
        # Assuming control_input is of shape (3, 1) with [v, w, dt]
        print("Control input in function:", control_input.shape)  # For debugging

        v = control_input[0, 0]  # Linear velocity
        w = control_input[1, 0]  # Angular velocity
        dt = control_input[2, 0]  # Time step

        # print("after state:", self.state)
        # print("dt:", dt)

        # B = np.array([[float(dt * np.cos(self.state[2])), 0],
        #             [float(dt * np.sin(self.state[2])), 0],
        #             [0, float(dt)]])

        B = np.array([[dt * np.cos(self.state[2, 0]), 0],
                [dt * np.sin(self.state[2, 0]), 0],
                [0, dt]])
        return B

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
