# pollibot_arm/pollination_manager_pub.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PollinationManagerPub(Node):
    def __init__(self):
        super().__init__('pollination_manager_pub')
        self.publisher_ = self.create_publisher(String, 'pollination_manager', 10)
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'pollination_manager_send!!'
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = PollinationManagerPub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
