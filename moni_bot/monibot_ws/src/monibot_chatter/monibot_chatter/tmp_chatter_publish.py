import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TestChatter(Node):
    def __init__(self):
        super().__init__('monibot_chatter_node')
        self.chatter_publisher1 = self.create_publisher(String,'send_chatter_to_somewhere',10)
        self.chatter_publisher2 = self.create_publisher(String,'send_chatter_to_somewhere_else',10)
        self.chatter_publisher3 = self.create_publisher(String,'send_chatter_to_somewhere_different',10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info("monibot_chatter_node has started. HA")

    def timer_callback(self):
        msg = String()

        self.send_msg(publisher=self.chatter_publisher1,msg=msg,data="Sending chatter from monibot_chatter_node publisher1 to somewhere")
        self.send_msg(publisher=self.chatter_publisher2,msg=msg,data="Sending chatter from monibot_chatter_node publisher2 to somewhere_else")
        self.send_msg(publisher=self.chatter_publisher3,msg=msg,data="Sending chatter from monibot_chatter_node publisher3 to somewhere_different")
        
        self.get_logger().info(f"chatter_publisher published")

    def send_msg(self,publisher,msg,data):
        msg.data = data
        publisher.publish(msg)


def main():
    rclpy.init()
    monibot_chatter_node = TestChatter()
    try:
        rclpy.spin(monibot_chatter_node)
    except KeyboardInterrupt:
        pass
    finally:
        monibot_chatter_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__=="__main__":
    main()