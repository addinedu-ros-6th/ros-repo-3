import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from rclpy.qos import QoSProfile

class MadgwickAHRS:
    def __init__(self, beta=0.1, sample_period=1.0/50):
        self.beta = beta
        self.sample_period = sample_period
        self.quaternion = np.array([1.0, 0.0, 0.0, 0.0])  # Initial orientation quaternion

    def update(self, gyroscope, accelerometer, magnetometer=None):
        acc = np.array(accelerometer)
        gyro = np.array(gyroscope)

        # Normalize accelerometer
        if np.linalg.norm(acc) == 0:
            return  # invalid acceleration vector
        acc /= np.linalg.norm(acc)

        # Rate of change of quaternion from gyroscope
        q_dot = 0.5 * self._quaternion_product(self.quaternion, np.array([0, gyro[0], gyro[1], gyro[2]]))

        # Gradient descent algorithm corrective step
        f = np.array([
            2 * (self.quaternion[1] * self.quaternion[3] - self.quaternion[0] * self.quaternion[2]) - acc[0],
            2 * (self.quaternion[0] * self.quaternion[1] + self.quaternion[2] * self.quaternion[3]) - acc[1],
            2 * (0.5 - self.quaternion[1]**2 - self.quaternion[2]**2) - acc[2]
        ])
        j = np.array([[
            -2 * self.quaternion[2], 2 * self.quaternion[3], -2 * self.quaternion[0], 2 * self.quaternion[1]
        ], [
            2 * self.quaternion[1], 2 * self.quaternion[0], 2 * self.quaternion[3], 2 * self.quaternion[2]
        ], [
            0, -4 * self.quaternion[1], -4 * self.quaternion[2], 0
        ]])
        step = np.dot(j.T, f)
        step /= np.linalg.norm(step)  # normalize step magnitude

        # Apply feedback step
        q_dot -= self.beta * step

        # Integrate to yield quaternion
        self.quaternion += q_dot * self.sample_period
        self.quaternion /= np.linalg.norm(self.quaternion)  # normalize quaternion

    def _quaternion_product(self, q, r):
        return np.array([
            q[0] * r[0] - q[1] * r[1] - q[2] * r[2] - q[3] * r[3],
            q[0] * r[1] + q[1] * r[0] + q[2] * r[3] - q[3] * r[2],
            q[0] * r[2] - q[1] * r[3] + q[2] * r[0] + q[3] * r[1],
            q[0] * r[3] + q[1] * r[2] - q[2] * r[1] + q[3] * r[0]
        ])

class IMUDataFilter(Node):
    def __init__(self):
        super().__init__('imu_data_filter')
        qos_profile = QoSProfile(depth=10)
        self.subscription = self.create_subscription(String, 'raw_imu', self.listener_callback, qos_profile)
        self.publisher = self.create_publisher(Imu, 'imu_data', qos_profile)
        self.madgwick = MadgwickAHRS(sample_period=1/50, beta=0.1)  # Adjust sample period and beta as needed

    def listener_callback(self, msg):
        # Assuming the message is in the format: "acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,mag_x,mag_y,mag_z"
        data = msg.data.split(',')

        try:
            # Parse the values from the string (update the indices based on your data format)
            acc_x, acc_y, acc_z = float(data[3]), float(data[4]), float(data[5])
            gyro_x, gyro_y, gyro_z = float(data[6]), float(data[7]), float(data[8])
            mag_x, mag_y, mag_z = float(data[9]), float(data[10]), float(data[11])

            # Update the Madgwick filter with the new data
            self.madgwick.update([gyro_x, gyro_y, gyro_z], [acc_x, acc_y, acc_z], [mag_x, mag_y, mag_z])

            # Retrieve the orientation quaternion from the Madgwick filter
            orientation = self.madgwick.quaternion

            # Create Imu message and populate fields
            imu_msg = Imu()
            imu_msg.linear_acceleration.x = acc_x
            imu_msg.linear_acceleration.y = acc_y
            imu_msg.linear_acceleration.z = acc_z
            imu_msg.angular_velocity.x = gyro_x
            imu_msg.angular_velocity.y = gyro_y
            imu_msg.angular_velocity.z = gyro_z

            # Set the orientation from the Madgwick filter
            imu_msg.orientation.w = orientation[0]
            imu_msg.orientation.x = orientation[1]
            imu_msg.orientation.y = orientation[2]
            imu_msg.orientation.z = orientation[3]

            # Publish the Imu message
            self.publisher.publish(imu_msg)

        except (ValueError, IndexError):
            self.get_logger().error('Failed to parse IMU data')

def main(args=None):
    rclpy.init(args=args)
    imu_data_filter = IMUDataFilter()
    try:
        rclpy.spin(imu_data_filter)
    finally:
        imu_data_filter.destroy_node()
        rclpy.shutdown()