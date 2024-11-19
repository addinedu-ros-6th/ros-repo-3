import re
import numpy as np
import rclpy
import serial
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from rclpy.qos import QoSProfile
import math

# Serial port configuration
comport_num = "/dev/ttyIMU"
comport_baudrate = 115200
try:
    ser = serial.Serial(port=comport_num, baudrate=comport_baudrate, timeout=0.001)  # 1 ms timeout
except serial.SerialException as e:
    print(f"Failed to connect to {comport_num} at {comport_baudrate}. Error: {e}")
    ser = None


class IMUDataFilter(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        qos_profile = QoSProfile(depth=10)
        self.publisher = self.create_publisher(Imu, 'imu', qos_profile)
        self.timer = self.create_timer(0.1, self.read_serial_data)  # 10 Hz (0.1s interval)

        # Initialize previous acceleration values for low-pass filter
        self.prev_accel_x = None
        self.prev_accel_y = None
        self.prev_accel_z = None

        # Low-pass filter parameters
        self.alpha = 0.1  # Filter strength, adjust as needed


    def read_serial_data(self):
        if ser and ser.in_waiting > 0:  # Only read if the serial connection is available
            try:
                # Read data from the serial port
                ser_data = ser.readline().decode('utf-8', errors='ignore').strip()
                
                # Check for valid data format
                if ser_data.startswith('*'):
                    # Use regex to extract data after '*'
                    match = re.match(r'\*(.+)', ser_data)

                    if match:
                        data_str = match.group(1)  # Extract data after the '*'
                        words = data_str.split(",")  # Split by commas

                        if len(words) != 13:
                            raise ValueError(f"Incorrect number of IMU data fields: {len(words)} found, expected 13. Data: {words}")

                        # Convert to float
                        data = list(map(float, words))

                        # Parse the values
                        quaternion_z, quaternion_y, quaternion_x, quaternion_w = data[0], data[1], data[2], data[3]
                        gyro_x, gyro_y, gyro_z = data[4], data[5], data[6]
                        acc_x, acc_y, acc_z = data[7], data[8], data[9]

                        # Convert linear acceleration from g to m/s^2
                        # acc_x = acc_x * 9.81  # Convert to m/s^2
                        # acc_y = acc_y * 9.81  # Convert to m/s^2
                        # acc_z = acc_z * 9.81  # Convert to m/s^2


                        # Convert gyro values from degrees/second to radians/second
                        gyro_x_rad = gyro_x * math.pi / 180
                        gyro_y_rad = gyro_y * math.pi / 180
                        gyro_z_rad = gyro_z * math.pi / 180


                        # Apply limits to linear velocity and acceleration values
                        acc_x = max(min(acc_x, 0.8), -0.8)  # Clamp acc_x between -8 and 0.8

                        # Apply limits to angular velocity values
                        gyro_z_rad = max(min(gyro_z_rad, 1.5), -1.5)  # Clamp gyro_z between -1.5 and 1.5

                        # Create and populate the Imu message
                        imu_msg = Imu()
                        imu_msg.header.stamp = self.get_clock().now().to_msg()
                        imu_msg.header.frame_id = 'imu_link'
                        imu_msg.linear_acceleration.x = acc_x
                        imu_msg.linear_acceleration.y = acc_y
                        imu_msg.linear_acceleration.z = acc_z #acc_z
                        imu_msg.angular_velocity.x = gyro_x_rad #gyro_x_rad
                        imu_msg.angular_velocity.y = gyro_y_rad #gyro_y_rad
                        imu_msg.angular_velocity.z = gyro_z_rad #gyro_z_rad
                        imu_msg.orientation.w = -quaternion_w #quaternion_w
                        imu_msg.orientation.x = -quaternion_x
                        imu_msg.orientation.y = -quaternion_y
                        imu_msg.orientation.z = -quaternion_z

                        # Publish the Imu message
                        self.publisher.publish(imu_msg)
                    else:
                        self.get_logger().error(f"No valid IMU data found in message: {ser_data}")
                else:
                    self.get_logger().warning(f"Invalid data format received: {ser_data}")
            except (ValueError, IndexError) as e:
                self.get_logger().error(f"Error while parsing IMU data: {str(e)}")
            except serial.SerialException as e:
                self.get_logger().error(f"Serial communication error: {str(e)}")
        elif not ser:
            self.get_logger().error("Serial connection is not initialized.")


def main(args=None):
    rclpy.init(args=args)
    imu_data_filter = IMUDataFilter()
    try:
        rclpy.spin(imu_data_filter)
    finally:
        imu_data_filter.destroy_node()
        rclpy.shutdown()
        if ser:
            ser.close()


if __name__ == '__main__':
    main()


