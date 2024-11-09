from rclpy.node import Node
import rclpy
from example_interfaces.msg import String
from example_interfaces.msg import Int32
from applecare_msgs.srv import DBCommand
from applecare_msgs.msg import TaskRequest
# from task_topic_subscriber_class import TaskTopicSubscriber
from datetime import datetime
import ast

class TaskManager(Node):
    def __init__(self):
        super().__init__('task_manager_node')
        self.task_subscriber = self.create_subscription(TaskRequest,'gui/task_topic',self.task_topic_callback,10)
        self.task_publisher_to_monibot1 = self.create_publisher(Int32,'monibot/task',10)
        self.task_publisher_to_pollibot1 = self.create_publisher(Int32,'pollibot/task',10)
        self.task_msg_data = None

        self.dbmanager_client = self.create_client(DBCommand,'/DB_server')
        while not self.dbmanager_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.DBrequest = DBCommand.Request()

    def task_topic_callback(self,msg):
        self.job_type = msg.job_type
        self.priority_area = msg.priority_area
        self.work_area = msg.work_area #a: 0, B:1, all: -1
        if self.work_area == 0:
            self.command_to_robot = 0 #A
        elif self.work_area == 1:
            self.command_to_robot = 1 #B
        elif self.work_area == -1:
            if self.priority_area == 0:
                self.command_to_robot = 2 #AB
            elif self.priority_area == 1:
                self.command_to_robot = 3 #BA 
            
        print("^^^^^^^^^^^^^^6")
        print(msg)
        print("^^^^^^^^^^^^4")

        # if self.job_type == 0: #scan
        #     robot_type = 0 # monibot
        # elif self.job_type == 1: #pollinate
        #     robot_type = 1 # polibot
        
        # if self.job_type in [0,1]: # get robot status when job_type is 0 scan, or 1 pollinate
        if msg.instant_task == True:

            robot_type = self.job_type
            cc = 0 # select
            table = "Robot r"
            column = "r.robot_id"

            where= f""" LEFT JOIN Task t ON r.robot_id = t.robot_id
                        where t.robot_id is null and r.robot_type = {robot_type}
                    """
            print("first_request!!!!!!")
            self.request_service(cc= 0, table=table, column=column,where=where)
            print("first request FINISHED@@@@@")

            # if self.task_msg_data == 'scan/A':
            #     cc = 0 #get
            #     table = "Robot r"
            #     column = "r.robot_id"

            #     where= """ LEFT JOIN Task t ON r.robot_id = t.robot_id
            #                 where t.robot_id is null and r.robot_type = 1;
            #             """
                                        # 우선 task table 수정해서 태스크 추가, 즉시면 먼저 state 가져와서 로봇 id까지 추가? 할당 시간도? 
                                        # 우선 state 가져오고,(task 에서 할당되지 않은 로봇 검색,)
                                        # 해당 아이디 시간 포함 task 등록
                                        # task 발행은 로봇 id 등록 되었을 때만.
                                        # A: 0 B:1 AB:2 BC:3
                
            # self.DBrequest = DBCommand.Request()
            # self.DBrequest.cc = cc
            # self.DBrequest.table = table
            # self.DBrequest.column = column
            # self.DBrequest.where = where
            
            # self.future = self.dbmanager_client.call_async(self.DBrequest)
            # self.future.add_done_callback(self.select_response_callback) # 비동기 방식
        elif msg.instant_task == False: # update values from first place
            task ="Task"
            self.request_service(cc=1,table="Task",
                                 column="(task_type, scheduled_time)",
                                 value = f"({self.job_type},'{msg.year}-{msg.month}-{msg.day} {msg.hour}:{msg.minute}:00')"
                                 )
            if self.command_to_robot == 0:
                tmp_value = "(LAST_INSERT_ID(),1,1)"
            elif self.command_to_robot == 1:
                tmp_value = "(LAST_INSERT_ID(),2,1)"
            elif self.command_to_robot == 2:
                tmp_value ="(LAST_INSERT_ID(),1,1), (LAST_INSERT_ID(),2,2)"
            elif self.command_to_robot == 3:
                tmp_value = "(LAST_INSERT_ID(),1,2), (LAST_INSERT_ID(),2,1)"
            self.request_service(cc=1,table="TaskRow",
                                 column="(task_id, row_id, sequence_no)",
                                 value=tmp_value

                                 )
    def select_response_callback(self,future):
        print("HELLO RESPONSE")
        try:
            response = future.result()
            if response is not None:
                self.response_data = response
                print("SAYCHEESES")
                print(self.response_data)
                if self.response_data.success == True:
                    print(self.response_data.success)
                    if self.response_data.result_list is not None:
                        self.result_list = [list(ast.literal_eval(item)) for item in self.response_data.result_list]
                        print(self.result_list)

                else:
                    print("Received empty response")
        except Exception as e:
            self.get_logger().error(f"Response call failed: {e}")

        self.publish_task(self.result_list)
    
    def publish_task(self,result_list):
        robot_id = result_list[0][0]

        task_command = Int32()
        task_command.data = self.command_to_robot

        if robot_id == 1:
            print("monibot1 publish")
            self.task_publisher_to_monibot1.publish(task_command)
            # cc = 1 # insert into
            # table = "Task"
            # column = "(task_type, robot_id, scheduled_time, actual_start_time)"
            # value = f"({self.job_type}, {robot_id},{datetime.now().replace(microsecond=0)},{datetime.now().replace(microsecond=0)})"
            
            # self.DBrequest = DBCommand.Request()
            # self.DBrequest.cc = cc
            # self.DBrequest.table = table
            # self.DBrequest.column = column
            # self.DBrequest.value = value

            # self.future_insert_task = self.dbmanager_client.call_async(self.DBrequest)#####)
            # self.future_insert_task.add_done_callback(self.insert_response_callback) # 비동기 방식
            self.request_service(cc=1,table="Task",
                                 column="(task_type, robot_id, scheduled_time, actual_start_time)",
                                 value = f"({self.job_type}, {robot_id},'{datetime.now().replace(microsecond=0)}','{datetime.now().replace(microsecond=0)}')"
                                 )
            if task_command.data == 0:
                tmp_value = "(LAST_INSERT_ID(),1,1)"
            elif task_command.data == 1:
                tmp_value = "(LAST_INSERT_ID(),2,1)"
            elif task_command.data == 2:
                tmp_value ="(LAST_INSERT_ID(),1,1), (LAST_INSERT_ID(),2,2)"
            elif task_command.data == 3:
                tmp_value = "(LAST_INSERT_ID(),1,2), (LAST_INSERT_ID(),2,1)"
            

            self.request_service(cc=1,table="TaskRow",
                                 column="(task_id, row_id, sequence_no)",
                                 value=tmp_value

                                 )
        

        if robot_id == 2:
            # task_command = Int32()
            # task_command.data = self.command_to_robot
            self.task_publisher_to_pollibot1.publish(task_command)
            print("&&&&&&&&&&&&&&7")
            print(task_command)
            print("%%%%%%%%%%%%%%%%")
        # 퍼블리시를 했으니 해당 task에 대해서 추가를 하거나, 로봇 아이디 할당. TaskRow도 업데이트 필요.
            
            self.request_service(cc=1,table="Task",
                                 column="(task_type, robot_id, scheduled_time, actual_start_time)",
                                 value = f"({self.job_type}, {robot_id},'{datetime.now().replace(microsecond=0)}','{datetime.now().replace(microsecond=0)}')"
                                 )
            if task_command.data == 0:
                tmp_value = "(LAST_INSERT_ID(),1,1)"
            elif task_command.data == 1:
                tmp_value = "(LAST_INSERT_ID(),2,1)"
            elif task_command.data == 2:
                tmp_value ="(LAST_INSERT_ID(),1,1), (LAST_INSERT_ID(),2,2)"
            elif task_command.data == 3:
                tmp_value = "(LAST_INSERT_ID(),1,2), (LAST_INSERT_ID(),2,1)"
            

            self.request_service(cc=1,table="TaskRow",
                                 column="(task_id, row_id, sequence_no)",
                                 value=tmp_value

                                 )
    def insert_response_callback(self,future):
        try:
            response = future.result()
            if response is not None:
                print(response)
                if response.success == True:
                    print("insert task success")
                else:
                    print("insert fail?")
        except Exception as e:
            print(f"Response call failed: {e}")




    def request_service(self,cc,table,column,where="",value=""):
        print("request function started!!!!!!s")
        self.DBrequest=DBCommand.Request()
        self.DBrequest.cc = cc
        self.DBrequest.table = table
        self.DBrequest.column = column
        self.DBrequest.where = where
        self.DBrequest.value = value
        print("DBrequest created!!!!!")
        if cc == 0:
            self.future_select = self.dbmanager_client.call_async(self.DBrequest)
            self.future_select.add_done_callback(self.select_response_callback)

        if cc == 1:
            self.future_insert = self.dbmanager_client.call_async(self.DBrequest)
            self.future_insert.add_done_callback(self.insert_response_callback)




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