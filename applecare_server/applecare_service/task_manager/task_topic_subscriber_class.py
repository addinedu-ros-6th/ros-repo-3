import rclpy as rp
from rclpy.node import Node

from std_msgs.msg import String

class TaskTopicSubscriber(Node):
    def __init__(self):
        super().__init__("task_topic_subscribe_node")
        self.task_topic_subscriber = self.create_subscription(String,'gui/task_topic',self.task_topic_callback,10)
        self.task_msg_data = None

        print("task_topic_sub_node has started")

    def task_topic_callback(self,msg):
        self.task_msg_data = msg.data
    
    def get_return_msg(self):
        # 현재 메시지를 임시 변수에 저장
        msg_data = self.task_msg_data
        # 메시지를 반환한 후 None으로 초기화
        self.return_msg_data = None
        return msg_data