# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import JointState
# import serial
# import time

# class MoveItMotorController(Node):
#     def __init__(self):
#         super().__init__('moveit_motor_controller')
#         self.subscription = self.create_subscription(
#             JointState,
#             '/joint_states',  # MoveIt에서 발행하는 관절 상태 토픽
#             self.joint_state_callback,
#             10
#         )
#         self.subscription  # prevent unused variable warning

#         # 시리얼 포트 설정
#         self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
#         self.last_sent_time = time.time()
#         # self.send_interval = 0.1  # 0.1초(100ms) 간격으로만 명령을 보냄
#         self.get_logger().info('MoveIt Motor Controller Node started')

#         # Joint positions을 저장할 리스트
#         self.joint_positions_list = []
#         self.max_samples = 50  # 50개 메시지를 받을 때까지 대기

#         # 시리얼 데이터 수집을 위한 리스트
#         self.serial_data = []

#     def joint_state_callback(self, msg):
#         # 각 메시지의 position 값을 리스트에 추가
#         joint_positions = msg.position
#         self.joint_positions_list.append(joint_positions)

#         # 50개의 메시지를 받은 경우
#         if len(self.joint_positions_list) >= self.max_samples:
#             # 평균 값을 계산하거나 처리
#             average_positions = [sum(pos) / len(pos) for pos in zip(*self.joint_positions_list)]
            
#             # 문자열로 변환하여 전송
#             command = ','.join([str(pos) for pos in average_positions])
#             self.serial_port.write(f"{command}\n".encode())

#             # 한 줄로 평균 값을 출력
#             self.get_logger().info(f"Sent joint positions: {command}")

#             # 리스트 초기화
#             self.joint_positions_list = []

#     def read_from_serial(self):
#         # 시리얼 데이터 읽기
#         if self.serial_port.in_waiting > 0:
#             line = self.serial_port.readline().decode().strip()
#             if line:
#                 if ", " in line:
#                     try:
#                         # Safely append the value after the "," delimiter
#                         self.serial_data.append(line.split(", ")[1])
#                     except IndexError:
#                         self.get_logger().warning(f"Unexpected format: {line}")
#                 else:
#                     self.get_logger().warning(f"Unexpected format: {line}")

#             # 모든 모터 데이터를 받은 경우 한 줄로 출력
#             if len(self.serial_data) == 4:
#                 combined_data = ', '.join(self.serial_data)
#                 self.get_logger().info(f"Received from Arduino: {combined_data}")
#                 self.serial_data = []  # 리스트 초기화

# def main(args=None):
#     rclpy.init(args=args)
#     moveit_motor_controller = MoveItMotorController()

#     try:
#         while rclpy.ok():
#             rclpy.spin_once(moveit_motor_controller, timeout_sec=0.1)
#             moveit_motor_controller.read_from_serial()
#     except KeyboardInterrupt:
#         pass

#     moveit_motor_controller.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()



# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import JointState
# import serial
# import time

# class MoveItMotorController(Node):
#     def __init__(self):
#         super().__init__('moveit_motor_controller')
#         self.subscription = self.create_subscription(
#             JointState,
#             '/joint_states',  # MoveIt에서 발행하는 관절 상태 토픽
#             self.joint_state_callback,
#             10
#         )
#         self.subscription  # prevent unused variable warning

#         # 시리얼 포트 설정
#         self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
#         self.last_sent_time = time.time()
#         self.send_interval = 0.1  # 0.1초(100ms) 간격으로만 명령을 보냄
#         self.get_logger().info('MoveIt Motor Controller Node started')

#         # Joint positions을 저장할 리스트
#         self.joint_positions_list = []
#         self.max_samples = 100  # 50개 메시지를 받을 때까지 대기

#     def joint_state_callback(self, msg):
#         # 각 메시지의 position 값을 리스트에 추가
#         joint_positions = msg.position
#         self.joint_positions_list.append(joint_positions)

#         # 50개의 메시지를 받은 경우
#         if len(self.joint_positions_list) >= self.max_samples:
#             # 평균 값을 계산하거나 처리
#             average_positions = [sum(pos) / len(pos) for pos in zip(*self.joint_positions_list)]
#             command = ','.join([str(pos) for pos in average_positions])
#             self.serial_port.write(f"{command}\n".encode())
#             self.get_logger().info(f'Sent joint positions: {command}')

#             # 리스트 초기화
#             self.joint_positions_list = []

#     def read_from_serial(self):
#         # 시리얼 데이터 읽기
#         if self.serial_port.in_waiting > 0:
#             line = self.serial_port.readline().decode().strip()
#             if line:
#                 self.get_logger().info(f'Received from Arduino: {line}')

# def main(args=None):
#     rclpy.init(args=args)
#     moveit_motor_controller = MoveItMotorController()

#     try:
#         while rclpy.ok():
#             rclpy.spin_once(moveit_motor_controller, timeout_sec=0.1)
#             moveit_motor_controller.read_from_serial()
#     except KeyboardInterrupt:
#         pass

#     moveit_motor_controller.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import JointState
# import serial
# import time

# class MoveItMotorController(Node):
#     def __init__(self):
#         super().__init__('moveit_motor_controller')
#         self.subscription = self.create_subscription(
#             JointState,
#             '/joint_states',  # MoveIt에서 발행하는 관절 상태 토픽
#             self.joint_state_callback,
#             10
#         )
#         self.subscription  # prevent unused variable warning

#         # 시리얼 포트 설정
#         self.serial_port = serial.Serial('/dev/ttyACM1', 115200, timeout=1)
#         self.last_sent_time = time.time()
#         # self.send_interval = 0.1  # 0.1초(100ms) 간격으로만 명령을 보냄
#         self.get_logger().info('MoveIt Motor Controller Node started')

#         # Joint positions을 저장할 리스트
#         self.joint_positions_list = []
#         self.max_samples = 50  # 10개 메시지를 받을 때까지 대기

#     def joint_state_callback(self, msg):
#         # 각 메시지의 position 값을 리스트에 추가
#         joint_positions = msg.position
#         self.joint_positions_list.append(joint_positions)

#         # 10개의 메시지를 받은 경우
#         if len(self.joint_positions_list) >= self.max_samples:
#             # 평균 값을 계산하거나 처리
#             average_positions = [sum(pos) / len(pos) for pos in zip(*self.joint_positions_list)]
#             command = ','.join([str(pos) for pos in average_positions])
#             self.serial_port.write(f"{command}\n".encode())
#             # self.get_logger().info(f'Sent joint positions: {command}')

#             # 리스트 초기화
#             self.joint_positions_list = []

#     def read_from_serial(self):
#         # 시리얼 데이터 읽기
#         if self.serial_port.in_waiting > 0:
#             line = self.serial_port.readline().decode().strip()
#             if line:
#                 self.get_logger().info(f'Received from Arduino: {line}')

# def main(args=None):
#     rclpy.init(args=args)
#     moveit_motor_controller = MoveItMotorController()
#     rclpy.spin(moveit_motor_controller)
#     moveit_motor_controller.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import serial
import time

class MoveItMotorController(Node):
    def __init__(self):
        super().__init__('moveit_motor_controller')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',  # MoveIt에서 발행하는 관절 상태 토픽
            self.joint_state_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        # 퍼블리셔 생성 (옵션)
        self.arduino_pub = self.create_publisher(String, 'arduino_data', 10)

        # 시리얼 포트 설정ttyMega2560
        self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        self.get_logger().info('MoveIt Motor Controller Node started')

        # Joint positions을 저장할 리스트
        self.joint_positions_list = []
        self.max_samples = 20  # 10개 메시지를 받을 때까지 대기

        # 시리얼 데이터 수집을 위한 리스트
        self.serial_data = []

        # 타이머 설정하여 시리얼 데이터 읽기
        # self.timer = self.create_timer(0.1, self.read_from_serial)  # 100ms 간격

    def joint_state_callback(self, msg):
        # 각 메시지의 position 값을 리스트에 추가
        joint_positions = msg.position
        self.joint_positions_list.append(joint_positions)

        # 4개의 메시지를 받은 경우
        if len(self.joint_positions_list) >= self.max_samples:
            # 평균 값을 계산
            average_positions = [sum(pos) / len(pos) for pos in zip(*self.joint_positions_list)]
            
            # 5개의 값만 선택 (필요한 관절 수에 맞게 조정)
            average_positions = average_positions[:5]
            
            # 문자열로 변환하여 전송
            command = ','.join([str(pos) for pos in average_positions])
            # command 소수점 6자리까지만 전송
            command = ','.join([f'{float(pos):.6f}' for pos in average_positions])

            self.serial_port.write(f"{command}\n".encode())
            self.get_logger().info(f'Sent joint positions: {command}')

            # 리스트 초기화
            self.joint_positions_list = []

    def read_from_serial(self):
        # 시리얼 데이터 읽기
        if self.serial_port.in_waiting > 0:
            line = self.serial_port.readline().decode().strip()
            if line:
                self.get_logger().info(f"Received from Arduino: {line}")
                # 퍼블리시 (옵션)
                # msg = String()
                # msg.data = line
                # self.arduino_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    moveit_motor_controller = MoveItMotorController()
    try:
        rclpy.spin(moveit_motor_controller)
    except KeyboardInterrupt:
        pass
    moveit_motor_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
