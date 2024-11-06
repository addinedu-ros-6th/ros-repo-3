from rclpy.node import Node
import rclpy
from applecare_msgs.srv import DBCommand # DB용 service에 정의필요
import mysql.connector

class DB_manager_server(Node):
    def __init__(self):
        super().__init__('db_manager')
        self.db_srv = self.create_service(DBCommand, 'DB_server',self.execute_command)
        self.db_srv # prevent malfunction
        self.command=None
        self.connection_to_database = False

        # login to applecare DB
        self.connect_to_database()

    # DB login function
    def connect_to_database(self):
        if not self.connection_to_database:
            self.connection =mysql.connector.connect(

                    # host='192.168.0.57',
                    # port = 3306, 
                    # user='lkm',
                    # password='1234',

                    host='localhost',
                    port = 3306, 
                    user='root',
                    password='1',

                    database='AppleCareDB'
                )
            self.cursor = self.connection.cursor()
            print(f"DB Connection Success")
            self.connection_to_database = True
        else:
            # print("already connected")
            pass

    def execute_command(self,request,response):
        # 만약 request 형태 DBCommand.srv 참조, string command
        cc=request.cc
        command = request.command
        if cc == 1:
            self.db_command_execute1(command)
        elif cc == "SOMETHING2":
            self.db_command_execute2()
        self.get_logger().info(f"DB 명령수신: {command}")

        response.success = True
        response.message = "DB 작업 성공"
        return response

    def db_command_execute1(self,command):
        print("db_execute1")
        # print(command)

    def db_command_execute2(self):
        print("db_execute2")

def main():
    try:
        rclpy.init()
        db_server = DB_manager_server()
        rclpy.spin(db_server)
    finally:
        db_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    
    