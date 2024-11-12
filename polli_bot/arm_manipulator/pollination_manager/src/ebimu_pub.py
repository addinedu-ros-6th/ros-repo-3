
#When the message "Permission denied: /dev/ttyx" appears, 
#change the permission settings on your serial_port
# eg. "sudo chmod 666 /dev/ttyUSB0"

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import Imu
import serial
import numpy

comport_num = "/dev/ttyUSB0"
comport_baudrate = 115200
# comport_num = '/dev/tty' + input("EBIMU Port: /dev/tty")
# comport_baudrate = input("Baudrate: ")
ser = serial.Serial(port=comport_num,baudrate=comport_baudrate)

try:
	ser = serial.Serial(port=comport_num, baudrate=comport_baudrate)
except:
	print('Serial port error!')


class EbimuPublisher(Node):

	def __init__(self):
		super().__init__('ebimu_publisher')
		qos_profile = QoSProfile(depth=10)
		qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT
		self.publisher = self.create_publisher(Imu, 'imu_data', qos_profile)
		# timer_period = 0.0005
		timer_period = 0.05
		self.timer = self.create_timer(timer_period, self.timer_callback)
		self.count = 0

	def timer_callback(self):
		msg = String()
		ser_data = ser.readline()
		msg.data = ser_data.decode('utf-8')
		self.publisher.publish(msg)		



def main(args=None):
	rclpy.init(args=args)

	print("Starting ebimu_publisher..")

	node = EbimuPublisher()

	try:
		rclpy.spin(node)

	finally:
		node.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main()


# --------------------------------------------------
# raw 9dof data from IMU

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from geometry_msgs.msg import Quaternion
import numpy as np
from madgwick_py import MadgwickAHRS
from rclpy.time import Time

class IMUPublisher(Node):
	def __init__(self):
		super().__init__('imu_publisher')
		qos_profile = QoSProfile(depth=10)
		qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT
		self.publisher = self.create_publisher(Imu, 'imu_data', qos_profile)
		self.subscription = self.create_subscription(String, 'raw_imu', self.listener_callback, qos_profile)
		self.madgwick = MadgwickAHRS(sample_period=1/50, beta=0.1)  # Adjust sample period and beta as needed

		"""
		sensor_.msgs.msg IMU data type contains header, orientation (quaternion), orientation  covariance,
		angular velocity (vector3), angular velocity covariance, linear acceleration (vector3), linear acceleration covariance
		https://docs.ros.org/en/lunar/api/sensor_msgs/html/msg/Imu.html

		"""

	def listener_callback(self, msg):
		data = msg.data.split(',')

		try:
			acc_x, acc_y, acc_z = float(data[0]), float(data[1]), float(data[2])
			gyro_x, gyro_y, gyro_z = float(data[3]), float(data[4]), float(data[5])
			mag_x, mag_y, mag_z = float(data[6]), float(data[7]), float(data[8])

			# Update the Madgwick filter with the new data
			self.madgwick.update([gyro_x, gyro_y, gyro_z], [acc_x, acc_y, acc_z], [mag_x, mag_y, mag_z])

			# Retrieve the orientation quaternion
			orientation = self.madgwick.quaternion

			# Create Imu message and populate fields
			imu_msg = Imu()
			
			# Header with timestamp and frame ID
			imu_msg.header.stamp = self.get_clock().now().to_msg()
			imu_msg.header.frame_id = 'imu_link'  # Set this to your IMU frame ID

			# Orientation
			imu_msg.orientation.w = orientation[0]
			imu_msg.orientation.x = orientation[1]
			imu_msg.orientation.y = orientation[2]
			imu_msg.orientation.z = orientation[3]
			imu_msg.orientation_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]  # Example covariance

			# Angular velocity
			imu_msg.angular_velocity.x = gyro_x
			imu_msg.angular_velocity.y = gyro_y
			imu_msg.angular_velocity.z = gyro_z
			imu_msg.angular_velocity_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]  # Example covariance

			# Linear acceleration
			imu_msg.linear_acceleration.x = acc_x
			imu_msg.linear_acceleration.y = acc_y
			imu_msg.linear_acceleration.z = acc_z
			imu_msg.linear_acceleration_covariance = [0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1]  # Example covariance

			# Publish the Imu message
			self.publisher.publish(imu_msg)

		except (ValueError, IndexError):
			self.get_logger().error('Failed to parse IMU data')

def main(args=None):
	rclpy.init(args=args)
	imu_publisher = IMUPublisher()
	rclpy.spin(imu_publisher)
	imu_publisher.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()


