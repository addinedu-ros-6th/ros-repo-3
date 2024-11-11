from rclpy.node import Node
import rclpy
from example_interfaces.msg import String
from applecare_msgs.msg import TaskRequest

class GuiTopicPublisher(Node):
    def __init__(self):
        super().__init__('gui_topic_publisher_node')
        self.gui_publishers = {
            'test_topic_name' : self.create_publisher(String, 'test_topic_name',10),
            'task_topic': self.create_publisher(TaskRequest, 'gui/task_topic',10),
            # 'pollinate_topic': self.create_publisher(String, 'pollinate_topic',10)
        }

    def publish_topic(self,topic_name,task_type,area,
                      priority_area=None,
                      instant_task=None,
                      A_sequence=None, B_sequence=None,
                      year=None, month=None, day=None,
                      hour=None,minute=None):
        if topic_name not in self.gui_publishers:
            # self.gui_publishers[topic_name] = self.create_publisher(String,topic_name,10)
            print(f"cannot publish /{topic_name}")
            return
        
        #이상한 예외처리...
        if area == None:
            if A_sequence == 1 and B_sequence == None:
                priority_area = 'A'
                area='A'
            elif A_sequence == None and B_sequence == 1:
                priority_area = 'B'
                area='B'
            elif A_sequence ==1 and B_sequence ==2:
                priority_area = "AB"
                area='All'
            elif A_sequence == 2 and B_sequence == 1:
                priority_area = "BA"   
                area='All'
        

        if priority_area == None:
            priority_area = 0 # priority A
        elif priority_area == 'A':
            priority_area = 0
        elif priority_area == 'B':
            priority_area = 1  # priority B
        elif priority_area == 'AB':
            priority_area = 0
        elif priority_area == 'BA':
            priority_area = 1

        if area == 'A':
            area = 0
        elif area == 'B':
            area = 1
        elif area == 'All':
            area = -1
        
        publish_topic = TaskRequest()
        publish_topic.job_type = task_type #string "scan/A" or "scan/AB"
        publish_topic.priority_area = priority_area
        publish_topic.work_area = area
        publish_topic.instant_task = True
        if instant_task == False:
            publish_topic.instant_task = instant_task
            publish_topic.year= year
            publish_topic.month= month
            publish_topic.day = day
            publish_topic.hour=hour
            publish_topic.minute=minute





        self.gui_publishers[topic_name].publish(publish_topic)
        print(publish_topic)