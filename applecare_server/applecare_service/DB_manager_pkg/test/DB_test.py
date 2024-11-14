import mysql.connector
import time
connection_to_database = False
if not connection_to_database:
    connection =mysql.connector.connect(

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
    cursor = connection.cursor()
    print(f"DB Connection Success")
    connection_to_database = True
else:
    # print("already connected")
    pass
test_flag = True
while True:
    # if test_flag==True:
    #     query="update Task set robot_id = null where task_id =36;"
    # else:
    #     query="update Task set robot_id = 1 where task_id =36;"
    # cursor.execute(query)
    # connection.commit()
    # test_flag = not test_flag
    cursor.execute("select robot_id from Task where task_id = 36;")
    db_result = cursor.fetchall()
    print(db_result)
    print("--------------------------")
    time.sleep(3)
