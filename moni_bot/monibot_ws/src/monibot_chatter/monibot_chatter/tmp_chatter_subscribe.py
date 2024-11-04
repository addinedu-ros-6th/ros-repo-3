import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TestChatterSub(Node):
    def __init__(self):
        super().__init__('monibot_chatter_sub_node')
        self.chatter_subscriber1 = self.create_subscription(String,'/ai_server',self.subscription_callback,10)
        self.get_logger().info("monibot_chatter_node_subscription has started. HAHA")

    def subscription_callback(self,msg):

        message = msg.data
        self.get_logger().info(f"received message by /send_chatter_to_somewhere : '{message}'")

def main():
    rclpy.init()
    monibot_chatter_node_sub = TestChatterSub()
    try:
        rclpy.spin(monibot_chatter_node_sub)
    except KeyboardInterrupt:
        pass
    finally:
        monibot_chatter_node_sub.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ =="__main__":
    main()


    