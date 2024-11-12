# pollibot_arm/pollination_manager_sub.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PollinationManagerSub(Node):
    def __init__(self):
        super().__init__('pollination_manager_sub')
        self.subscription = self.create_subscription(
            String,
            'pollitask_service',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
        self.get_logger().info('Received from pollitask_service "%s"' % msg.data )

def main(args=None):
    rclpy.init(args=args)
    node = PollinationManagerSub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

