import rclpy as rp
from rclpy.node import Node
from std_msgs.msg import String

class ai_server_subscriber(Node):
    def __init__(self):
        super().__init__('ai_server_subscriber')
        self.pollination_subscription = self.create_subscription(String, 'pollination_manager', self.callback, 10)
        self.obstacle_subscription = self.create_subscription(String, 'moni_bot_controller', self.callback, 10)
        self.pollination_subscription
        self.obstacle_subscription

    def callback(self, msg):
        print("receive message: ", msg)

def main(args=None):
    rp.init(args=args)

    ai_server_subscrib = ai_server_subscriber()
    rp.spin(ai_server_subscrib)

    ai_server_subscrib.destroy_node()
    rp.shutdown()

if __name__ == "__main__":
    main()