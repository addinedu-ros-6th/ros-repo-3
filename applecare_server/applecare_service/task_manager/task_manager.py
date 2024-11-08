from rclpy.node import Node
import rclpy
from example_interfaces.msg import String
from example_interfaces.msg import Int32
from applecare_msgs.srv import DBCommand
from task_topic_subscriber_class import TaskTopicSubscriber

import ast

class TaskManager(Node):
    def __init__(self,gui_task_subscriber_node):
        super().__init__('task_manager_node')
        self.gui_task_subscriber_node = gui_task_subscriber_node
        # self.task_subscriber = self.create_subscription(String,'gui/task_topic',self.task_topic_callback,10)
        self.task_publisher_to_pollibot = self.create_publisher(Int32,'pollibot/task',10)
        self.task_msg_data = None

        self.dbmanager_client = self.create_client(DBCommand,'/DB_server')
        while not self.dbmanager_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.DBrequest = DBCommand.Request()

    def task_topic_callback(self,msg):
        self.task_msg_data = msg.data
        print(self.task_msg_data)
        if self.task_msg_data == 'scan/A':
            cc = 0 #get
            table = "Robot r"
            column = "r.robot_id"

            where= """ LEFT JOIN Task t ON r.robot_id = t.robot_id
                        where t.robot_id is null and r.robot_type = 1;
                    """
                                    # 우선 task table 수정해서 태스크 추가, 즉시면 먼저 state 가져와서 로봇 id까지 추가? 할당 시간도? 
                                    # 우선 state 가져오고,(task 에서 할당되지 않은 로봇 검색,)
                                    # 해당 아이디 시간 포함 task 등록
                                    # task 발행은 로봇 id 등록 되었을 때만.
            
        # self.DBrequest = DBCommand.Request()
        self.DBrequest.cc = cc
        self.DBrequest.table = table
        self.DBrequest.column = column
        self.DBrequest.where = where
        
        self.future = self.dbmanager_client.call_async(self.DBrequest)
        # self.future.add_done_callback(self.response_callback) # 비동기 방식
        # print(1)
        # self.future = self.dbmanager_client.call(self.DBrequest)
        # print(2)
        # self.response_callback(self.future)  # 동기 방식이므로 호출 결과를 바로 전달
        # print(3)
        # import time
        # while self.future.result() == None:
        #     print("^^^^^^^^^^^^")
        #     print(self.future.result())
        #     # time.sleep(1)
        #     rclpy.spin_once(self,timeout_sec=1)
        print(1)
        rclpy.spin_until_future_complete(self, self.future)  # 응답을 기다리기 위해 spin을 추가
        print(2)
        self.response_callback(self.future)

        self.robot_id = self.result_list[0][0]

        if self.robot_id == 2:
            task_command = Int32()
            task_command.data = 0
            self.task_publisher_to_pollibot.publish(task_command)


        print("task_command /monibot_scan published... maybe")

    def response_callback(self,future):
        print("HELLO RESPONSE")
        try:
            response = future.result()
            if response is not None:
                self.response_data = response
                print(self.response_data)
                if self.response_data.success == True:

                    # robot_id = self.response_data.data
                    # robot_id = 2 # data from db_manager response
                    # task_topic = 'scan' # data from task_subscriber
                    # work_area = 'all'   # data from task_subscriber
                    # task_command = Int32()
                    self.result_list = [list(ast.literal_eval(item)) for item in self.response_data.result_list]
                    print(self.result_list)
                    # task_command.data = "0"
                    # self.task_publisher.publish(task_command)
                else:
                    print("Received empty response")
        except Exception as e:
            self.get_logger().error(f"Response call failed: {e}")


def main():
    rclpy.init()
    
    task_manager_node = TaskManager()
    
    try:
        rclpy.spin(task_manager_node) 
    except KeyboardInterrupt:
        pass
    finally:
        task_manager_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
                # 대기중인 로봇 정보 받아서 그 로봇에게 스캔 작업 시작 주기.


        # self.gui_publishers = {
        #     'test_topic_name' : self.create_publisher(String, 'test_topic_name',10),
        #     'scan_topic': self.create_publisher(String, 'scan_topic',10),
        #     'pollinate_topic': self.create_publisher(String, 'pollinate_topic',10)
        # }

    # def publish_topic(self,topic_name,task_type,area):
    #     if topic_name not in self.gui_publishers:
    #         # self.gui_publishers[topic_name] = self.create_publisher(String,topic_name,10)
    #         print(f"cannot publish /{topic_name}")
    #         return
    #     if task_type == 'scan':
    #         publish_topic = String()
    #         publish_topic.data = f"scan/{area}"

    #     if task_type == 'pollinate':
    #         publish_topic = String()
    #         publish_topic.data = f"pollinate/{area}"
    #     self.gui_publishers[topic_name].publish(publish_topic)