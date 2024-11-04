import rclpy as rp
from rclpy.node import Node
from std_msgs.msg import String

class ai_server_publisher(Node):
    def __init__(self):
        super().__init__('ai_server_publisher')
        self.publisher = self.create_publisher(String, 'ai_server', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
    
    def timer_callback(self):
        msg = String()
        msg.data = "ai_server send message"
        self.publisher.publish(msg=msg)

        self.get_logger().info(f"{msg.data}")

def main(args=None):
    rp.init(args=args)

    ai_server_publish = ai_server_publisher()
    rp.spin(ai_server_publish)

    ai_server_publish.destroy_node()
    rp.shutdown()

if __name__ == "__main__":
    main()