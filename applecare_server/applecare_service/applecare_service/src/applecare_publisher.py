#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class AppleCarePublisher(Node):
    def __init__(self):
        super().__init__('applecare_publisher')
        self.publisher_ = self.create_publisher(String, 'applecare_service', 10)
        timer_period = 0.5  # 발행 주기 (초)
        self.timer = self.create_timer(timer_period, self.publish_message)
        self.get_logger().info('AppleCare Publisher Node has been started.')

    def publish_message(self):
        msg = String()
        msg.data = 'applecare_service_send'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = AppleCarePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
