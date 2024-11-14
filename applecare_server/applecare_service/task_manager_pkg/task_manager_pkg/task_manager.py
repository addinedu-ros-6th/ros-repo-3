from rclpy.node import Node
import rclpy
from example_interfaces.msg import String
from std_msgs.msg import Int32
# from applecare_msgs.srv import DBCommand
# from applecare_msgs.msg import TaskRequest

from gui_manager_msgs.srv import DBCommand
from gui_manager_msgs.msg import TaskRequest
# from task_topic_subscriber_class import TaskTopicSubscriber
from datetime import datetime
import ast
import re
import time
from datetime import datetime
class TaskManager(Node):
    def __init__(self):
        super().__init__('task_manager_node')
        self.task_subscriber = self.create_subscription(TaskRequest,'gui/task_topic',self.task_topic_callback,10)
        self.task_publisher_to_monibot1 = self.create_publisher(Int32,'monibot/task',10)
        # self.task_publisher_to_pollibot1 = self.create_publisher(Int32,'pollibot/task',10)
        self.task_publisher_to_pollibot1 = self.create_publisher(Int32,'zone_selection',10)
        self.task_msg_data = None

        self.dbmanager_client = self.create_client(DBCommand,'/DB_server')
        while not self.dbmanager_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.DBrequest = DBCommand.Request()
        # self.get_scheduled_task()
        self.timer2 = self.create_timer(10, self.get_scheduled_task)
        self.timer = self.create_timer(30.0, self.check_scheduled_time)
        # self.check_scheduled_time()
    def get_scheduled_task(self):
        print("$$%$%$%$")
        query = """ select 
                        t.task_id, t.task_type, tr.row_id, tr.sequence_no, t.scheduled_time 
                    from 
                        Task t 
                    left join 
                        TaskRow tr on t.task_id = tr.task_id 
                    where 
                        scheduled_time > now();
                    """ 
        tempResquest = DBCommand.Request()
        tempResquest.cc = 0
        tempResquest.query =query
        self.schedule_future =self.dbmanager_client.call_async(tempResquest)
        self.schedule_future.add_done_callback(self.schedule_response_callback)
        ###아마 이렇게 나올거야!!!!!!!
    def schedule_response_callback(self,future):
        response = future.result()
        self.schedule_data_list = []
        for item in response.result_list:
            item = re.sub(r'(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2})', r"'\1'", item)
            parsed_item = ast.literal_eval(item)
            if isinstance(parsed_item, int):
                # 정수일 경우 리스트로 감싸서 추가
                self.schedule_data_list.append([parsed_item])
            else:
                # 이미 튜플인 경우 그대로 리스트로 변환
                self.schedule_data_list.append(list(parsed_item))
        print(self.schedule_data_list)
        # [[1, 0, 1, 1, '2099-12-31 08:00:00'], 
        # [1, 0, 2, 2, '2099-12-31 08:00:00'], 
        # [2, 1, 2, 1, '2099-11-09 10:00:00']]
        # self.check_scheduled_time()
    def check_scheduled_time(self):
        # 현재 시간 읽기
        current_time = datetime.now()
        # formatted_time = '2099-11-19 10:00:00'
        formatted_time = current_time.strftime('%Y-%m-%d %H:%M:%S')
        # if formatted_time > '2024-11-14 14:30:50':
        #     print("heyeeyeyy")
        try:
            for list in self.schedule_data_list:
                print(list)
                if list[-1] < formatted_time:
                    print("current list = ",list)
                    print("scheduled before!!")
                    self.scheduled_task_id = list[0]
                    print("self.scheduled_task_id = ", self.scheduled_task_id)
                    task_type = list[1]
                    # data = [item for item in self.schedule_data_list if item[0] != task_id]
                    filtered_values = [(item[2], item[3]) for item in self.schedule_data_list if item[0] == self.scheduled_task_id]
                    print("filtered_values = ",filtered_values)
                    if filtered_values:
                        if len(filtered_values) == 2:
                            if filtered_values[0][0] == 1 and filtered_values[0][1]==1:
                                pub = 2
                            else:
                                pub = 3
                        else:
                            if filtered_values[0][0] == 1:
                                pub = 0
                            else:
                                pub = 1
                        self.schedule_to_robot = pub
                        self.get_robot_id_publish_task(task_type)

                    # task_type = list[1]
                    # row_id = list[2]
                    # sequence 
                    print("self.scheduled_task_id =", self.scheduled_task_id)
                    self.schedule_data_list = [item for item in self.schedule_data_list if item[0] != self.scheduled_task_id]
            print("현재 시간:", formatted_time)
        except:
            pass
    def get_robot_id_publish_task(self,task_type):
        robot_type = task_type
        query = f"""select r.robot_id 
                        from Robot r 
                        LEFT JOIN Task t ON r.robot_id = t.robot_id
                        where t.robot_id is null and r.robot_type = {robot_type};
                    """
        command = DBCommand.Request()
        command.cc = 0
        command.query = query
        self.get_robot_id_schedule_future = self.dbmanager_client.call_async(command)
        self.get_robot_id_schedule_future.add_done_callback(self.publish_task_scheduled)
    def publish_task_scheduled(self,future):
        try:
            response = future.result()
            if response is not None:
                if response.success == True:
                    if response.result_list is not None:
                        # self.result_list = [list(ast.literal_eval(item)) for item in self.response_data.result_list]
                        # print(self.result_list) # DB manager 에서 바로 [[1,]]이런식으로 나오도록 했는데 에러 안뜰려나?(11.11)
                                                # 괜찮은거 같음 (11.12) #응 안돼 (11.13)
                        result_list = []
                        for item in response.result_list:
                            parsed_item = ast.literal_eval(item)
                            if isinstance(parsed_item, int):
                                # 정수일 경우 리스트로 감싸서 추가
                                result_list.append([parsed_item])
                            else:
                                # 이미 튜플인 경우 그대로 리스트로 변환
                                result_list.append(list(parsed_item))
                        print(result_list)
                        robot_id=result_list[0][0]
                        robot_command = Int32()
                        robot_command.data = self.schedule_to_robot
                        if robot_id == 1:
                            self.task_publisher_to_monibot1.publish(robot_command)
                        elif robot_id ==2:
                            self.task_publisher_to_pollibot1.publish(robot_command)
                        query = f""" update Task set robot_id = {robot_id}, actual_start_time = now() 
                                        where task_id = {self.scheduled_task_id}
                                        """
                        command = DBCommand.Request()
                        command.cc=1
                        command.query=query
                        self.dbmanager_client.call_async(command)
                        print("123456512344565")



                else:
                    print("Received empty response")
        except Exception as e:
            self.get_logger().error(f"Response call failed: {e}")



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
            
            query = f"""select r.robot_id 
                        from Robot r 
                        LEFT JOIN Task t ON r.robot_id = t.robot_id
                        where t.robot_id is null and r.robot_type = {robot_type};
                        """
            print("first_request!!!!!!")
            self.request_service(cc= 0, query=query)
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
            query = f""" insert into Task (task_type, scheduled_time)
                        values ({self.job_type},'{msg.year}-{msg.month}-{msg.day} {msg.hour}:{msg.minute}:00');
                        """
            self.request_service(cc=1,query=query)
            if self.command_to_robot == 0:
                tmp_value = "(LAST_INSERT_ID(),1,1)"
            elif self.command_to_robot == 1:
                tmp_value = "(LAST_INSERT_ID(),2,1)"
            elif self.command_to_robot == 2:
                tmp_value ="(LAST_INSERT_ID(),1,1), (LAST_INSERT_ID(),2,2)"
            elif self.command_to_robot == 3:
                tmp_value = "(LAST_INSERT_ID(),1,2), (LAST_INSERT_ID(),2,1)"

            query = f""" insert into TaskRow (task_id, row_id, sequence_no)
                        values {tmp_value};
                        """
            self.request_service(cc=1,query=query)
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
                        # self.result_list = [list(ast.literal_eval(item)) for item in self.response_data.result_list]
                        # print(self.result_list) # DB manager 에서 바로 [[1,]]이런식으로 나오도록 했는데 에러 안뜰려나?(11.11)
                                                # 괜찮은거 같음 (11.12) #응 안돼 (11.13)
                        self.result_list = []
                        for item in self.response_data.result_list:
                            parsed_item = ast.literal_eval(item)
                            if isinstance(parsed_item, int):
                                # 정수일 경우 리스트로 감싸서 추가
                                self.result_list.append([parsed_item])
                            else:
                                # 이미 튜플인 경우 그대로 리스트로 변환
                                self.result_list.append(list(parsed_item))
                        print("hey its me1",self.result_list)

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
            query = f""" insert into Task (task_type, robot_id, scheduled_time, actual_start_time)
                        values ({self.job_type}, {robot_id},'{datetime.now().replace(microsecond=0)}','{datetime.now().replace(microsecond=0)}');
                        """

            self.request_service(cc=1,query=query)
            if task_command.data == 0:
                tmp_value = "(LAST_INSERT_ID(),1,1)"
            elif task_command.data == 1:
                tmp_value = "(LAST_INSERT_ID(),2,1)"
            elif task_command.data == 2:
                tmp_value ="(LAST_INSERT_ID(),1,1), (LAST_INSERT_ID(),2,2)"
            elif task_command.data == 3:
                tmp_value = "(LAST_INSERT_ID(),1,2), (LAST_INSERT_ID(),2,1)"
            
            query = f""" insert into TaskRow (task_id, row_id, sequence_no)
                        values {tmp_value};
                        """
            self.request_service(cc=1,query=query)
        

        if robot_id == 2:
            task_command = Int32()
            task_command.data = self.command_to_robot
            while not self.task_publisher_to_pollibot1.get_subscription_count():
                rclpy.spin_once(self, timeout_sec=0.1)
                time.sleep(0.5)
                print("hey its me2",self.task_publisher_to_pollibot1.get_subscription_count())
            self.task_publisher_to_pollibot1.publish(task_command)
            self.task_publisher_to_pollibot1.wait_for_all_acked()
            # for i in range(20):
            #     self.task_publisher_to_pollibot1.publish(task_command)
            #     print(task_command)
            #     time.sleep(0.2)

            print("&&&&&&&&&&&&&&7")
            print(task_command)
            print("%%%%%%%%%%%%%%%%")
        # 퍼블리시를 했으니 해당 task에 대해서 추가를 하거나, 로봇 아이디 할당. TaskRow도 업데이트 필요.
            
            query = f""" insert into Task (task_type, robot_id, scheduled_time, actual_start_time)
                        values ({self.job_type}, {robot_id},'{datetime.now().replace(microsecond=0)}','{datetime.now().replace(microsecond=0)}');
                        """

            self.request_service(cc=1,query=query)
            if task_command.data == 0:
                tmp_value = "(LAST_INSERT_ID(),1,1)"
            elif task_command.data == 1:
                tmp_value = "(LAST_INSERT_ID(),2,1)"
            elif task_command.data == 2:
                tmp_value ="(LAST_INSERT_ID(),1,1), (LAST_INSERT_ID(),2,2)"
            elif task_command.data == 3:
                tmp_value = "(LAST_INSERT_ID(),1,2), (LAST_INSERT_ID(),2,1)"
            
            query = f""" insert into TaskRow (task_id, row_id, sequence_no)
                        values {tmp_value};
                        """
            self.request_service(cc=1,query=query)
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




    def request_service(self,cc,query):
        print("request function started!!!!!!s")
        self.DBrequest=DBCommand.Request()
        self.DBrequest.cc = cc
        self.DBrequest.query = query
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
                # 대기중인 로봇 정보 받아서 그 로봇에게 스캔 작업 시작 주기. (done)


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