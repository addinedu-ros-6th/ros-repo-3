import rclpy
from rclpy.node import Node
from applecare_msgs.srv import DBCommand

class DBCommandClient(Node):
    def __init__(self):
        super().__init__('db_command_client')
        self.client = self.create_client(DBCommand, 'DB_server')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('서비스를 기다리는 중...')

    def send_command(self, cc, command):
        request = DBCommand.Request()
        request.cc = cc
        request.command = command
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info(f'응답: {future.result().message}')
        else:
            self.get_logger().error('DB 명령 처리 실패')

def main(args=None):
    try:
        rclpy.init(args=args)
        client = DBCommandClient()
        client.send_command(1,"CCINSERT INTO table_name VALUES ('value1', 'value2')")
    finally:
        client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
