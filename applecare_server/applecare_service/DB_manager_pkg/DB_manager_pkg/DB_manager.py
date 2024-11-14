from rclpy.node import Node
import rclpy
from applecare_msgs.srv import DBCommand # DB용 service에 정의필요
# from gui_manager_msgs.srv import DBCommand
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

                    # database='AppleCareDB'
                    database='FarmManagement'
                )
            # self.cursor = self.connection.cursor(buffer=True)
            self.cursor = self.connection.cursor()
            print(f"DB Connection Success")
            self.connection_to_database = True
        else:
            # print("already connected")
            pass

    def execute_command(self,request,response):
        # 만약 request 형태 DBCommand.srv 참조, string command
        # -------------------------------------
        # tmp request data
        # request = DBCommand.Request()
        # request.cc = 0
        # request.table = "RobotStatus"
        # request.column = "status"
        # request.where = "where robot_id = 2 and robot_type = 'moni'"
        # -------------------------------------

        cc=request.cc 
        # table = request.table
        # column = request.column
        # where = request.where
        # value = request.value #sql query
        query = request.query
        if cc == 0:
            db_result=self.search_from_db(query)
        elif cc == 1:
            db_result=self.insert_to_db(query)
        # self.get_logger().info(f"DB 명령수신: {command}")

        print("#########################")
        print(db_result)

        response.success = True
        result_list = []
        # if db_result is not None:
        #     for result in db_result:
        #         result_list.append(str(result))
        if db_result is not None:
        # Decimal 값을 문자열로 변환하여 "(1,0)" 형태로 result_list에 추가
            for result in db_result:
                result_str = f"({','.join(str(item) for item in result)})"  # db의 데이터 타입(Decimal)을 문자열로 변환, 안하면 (Decimal(1),Decimal(2)) 이렇게 나옴
                result_list.append(result_str)
        print("%%%%%%%%%%%%%%%%%")
        print(result_list)
        response.result_list = result_list
        print(response)
        return response

    def search_from_db(self,query):
        print("search from db")
        print(query)
        self.cursor.execute(query)
        # self.cursor.execute("select * from Task")
        db_result = self.cursor.fetchall()
        print("%%%%%%%%%%%55")
        print(db_result)
        print("$$$$$$$$$$$")
        return db_result
        # print(command)

    def insert_to_db(self,query):
        print("insert to db")
        self.cursor.execute(query)
        self.connection.commit()
        return None

def main():
    try:
        rclpy.init()
        db_server = DB_manager_server()
        rclpy.spin(db_server)
    finally:
        db_server.connection.close()
        db_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    
    
