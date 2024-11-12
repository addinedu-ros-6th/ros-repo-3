import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile
import serial

comport_num = "/dev/ttyIMU"
comport_baudrate = 115200
ser = serial.Serial(port=comport_num, baudrate=comport_baudrate)

class IMURawPublisher(Node):
    def __init__(self):
        super().__init__('imu_raw_publisher')
        qos_profile = QoSProfile(depth=10)
        self.raw_publisher = self.create_publisher(String, 'raw_imu', qos_profile)

        # Start a timer to read from the serial port periodically
        self.timer = self.create_timer(0.02, self.read_serial_data)  # 50 Hz rate

    def read_serial_data(self):
        try:
            ser_data = ser.readline()
            msg = String()
            # msg.data = ser_data.decode('utf-8').strip()
            msg.data = ser_data.decode('utf-8', errors='ignore').strip()  # Ignore problematic characters

            self.raw_publisher.publish(msg)  # Publish raw IMU data to the 'raw_imu' topic
        # except serial.SerialException as e:
        #     self.get_logger().error(f"Serial read error: {e}")\
        except UnicodeDecodeError:
            self.get_logger().error(f"decoding error: {e}")
            return

def main(args=None):
    rclpy.init(args=args)
    imu_raw_publisher = IMURawPublisher()
    try:
        rclpy.spin(imu_raw_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        imu_raw_publisher.destroy_node()
        rclpy.shutdown()
