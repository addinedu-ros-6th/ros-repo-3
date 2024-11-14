#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MultiSubscriber(Node):
    def __init__(self):
        super().__init__('multi_subscriber')
        
        # 구독할 토픽 목록
        topics = {
            'pollitast_service': self.pollitast_callback,
            'ai_server': self.ai_server_callback,
            'moni_bot_controller': self.moni_bot_callback
        }
        
        # 각 토픽에 대한 구독자 생성
        for topic_name, callback in topics.items():
            self.create_subscription(
                String,
                topic_name,
                callback,
                10
            )
            self.get_logger().info(f'Subscribed to {topic_name}')
    
    def pollitast_callback(self, msg):
        self.get_logger().info(f'Received from pollitast_service: "{msg.data}"')
    
    def ai_server_callback(self, msg):
        self.get_logger().info(f'Received from ai_server: "{msg.data}"')
    
    def moni_bot_callback(self, msg):
        self.get_logger().info(f'Received from moni_bot_controller: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = MultiSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
