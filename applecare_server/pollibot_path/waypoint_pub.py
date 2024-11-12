#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from std_msgs.msg import Bool, Int32, String
import numpy as np
import math
from tf_transformations import quaternion_from_euler
import time

class WaypointPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_publisher')

        # 퍼블리셔 생성
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.goal_pose_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.goal_stop_pub = self.create_publisher(Bool, '/goal_stop', 10)  # 정지 여부 퍼블리셔
        self.obstacle_stop_flag_pub = self.create_publisher(Bool, '/obstacle_stop_flag', 10)  # 장애물 정지 퍼블리셔 추가

        # 구독자 생성
        self.goal_reached_sub = self.create_subscription(PoseStamped, '/goal_reached', self.goal_reached_callback, 10)
        self.zone_selection_sub = self.create_subscription(Int32, '/zone_selection', self.zone_selection_callback, 10)
        self.get_logger().info("구역 선택 구독자 생성 완료.")

        # obstacle_stop 구독자 생성
        self.obstacle_stop_sub = self.create_subscription(String, '/obstacle_stop', self.obstacle_stop_callback, 10)
        self.get_logger().info("obstacle_stop 구독자 생성 완료.")

        # 정지 상태 변수 추가
        self.is_stopped = False

        # 웨이포인트 로드
        self.load_waypoints()

        # 메인 경로 정의
        self.paths = {
            0: {'name': 'Path_A', 'waypoints': self.initial_path + self.path_to_A + self.A_waypoints + self.return_path},
            1: {'name': 'Path_B', 'waypoints': self.initial_path + self.path_to_B + self.B_waypoints + self.return_path},
            2: {'name': 'Path_A_to_B', 'waypoints': self.initial_path + self.path_to_A + self.A_waypoints + self.path_to_B + self.B_waypoints + self.return_path},
            3: {'name': 'Path_B_to_A', 'waypoints': self.initial_path + self.path_to_B + self.B_waypoints + self.path_to_A + self.A_waypoints + self.return_path}
        }
        self.get_logger().info(f"정의된 경로: {list(self.paths.keys())}")

        # 초기 상태 설정
        self.selected_path = None  # 초기에는 경로가 선택되지 않음

        # 웨이포인트 퍼블리싱 상태 초기화
        self.current_waypoint_index = 0
        self.is_waiting_for_robot = False
        self.current_goal_pose = None
        self.all_waypoints_published = False  # 모든 웨이포인트 퍼블리시 여부 플래그
        self.goal_tolerance = 0.5  # 허용 오차 범위 설정 (필요에 따라 조정)

        # 초기 위치 퍼블리시
        self.publish_initial_pose()
        self.get_logger().info("초기 위치 퍼블리시 완료. 2초 대기 후 구역 선택 대기.")
        self.start_time = self.get_clock().now()

        # 초기 대기 타이머 설정 (2초 후 초기화)
        self.initial_timer = self.create_timer(2.0, self.initial_timer_callback)

        # 웨이포인트 퍼블리싱 타이머는 구역 선택 후에만 생성
        self.timer = None

    def load_waypoints(self):
        try:
            waypoint_file = '/home/leesiwon/main_server/src/turtlebot4/turtlebot4_navigation/waypoints_extended.npy'
            data = np.load(waypoint_file, allow_pickle=True).item()
            self.initial_pose = data['initial_pose']
            self.initial_path = data.get('initial_path', [])
            self.path_to_A = data.get('path_to_A', [])
            self.A_waypoints = data.get('A_waypoints', [])
            self.path_to_B = data.get('path_to_B', [])
            self.B_waypoints = data.get('B_waypoints', [])
            self.return_path = data.get('return_path', [])
            self.get_logger().info("확장된 웨이포인트를 성공적으로 로드했습니다.")
        except Exception as e:
            self.get_logger().error(f"웨이포인트를 로드하는 데 실패했습니다: {e}")
            rclpy.shutdown()

    def publish_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = self.initial_pose[0]
        msg.pose.pose.position.y = self.initial_pose[1]
        msg.pose.pose.position.z = 0.0

        # 첫 번째 웨이포인트가 있으면 방향 설정
        if self.initial_path and len(self.initial_path) > 0:
            dx = self.initial_path[0]['pose'][0] - self.initial_pose[0]
            dy = self.initial_path[0]['pose'][1] - self.initial_pose[1]
            yaw = math.atan2(dy, dx)
        else:
            yaw = 0.0

        quaternion = quaternion_from_euler(0, 0, yaw)
        msg.pose.pose.orientation.x = quaternion[0]
        msg.pose.pose.orientation.y = quaternion[1]
        msg.pose.pose.orientation.z = quaternion[2]
        msg.pose.pose.orientation.w = quaternion[3]

        # 공분산 설정 (필요 시 조정)
        msg.pose.covariance = [0.0]*36

        # 여러 번 퍼블리시하여 초기 위치가 확실히 전달되도록 함
        for _ in range(10):
            self.initial_pose_pub.publish(msg)
            self.get_logger().info("초기 위치 퍼블리시")
            time.sleep(0.1)  

    def initial_timer_callback(self):
        # 초기 대기 타이머 콜백: 대기 후 타이머 취소
        self.initial_timer.cancel()
        self.get_logger().info("초기 대기 완료. 구역 선택 메시지 대기 중.")

    def timer_callback(self):
        # 모든 웨이포인트가 퍼블리시되었으면 타이머 종료
        if self.all_waypoints_published:
            self.get_logger().info("모든 웨이포인트 퍼블리시 완료. 타이머 중지.")
            self.timer.cancel()
            return

        # 경로가 선택되지 않았으면 대기
        if self.selected_path is None:
            self.get_logger().warn("선택된 경로가 없습니다. 경로 선택을 기다립니다.")
            return

        # 웨이포인트 퍼블리싱
        if self.current_waypoint_index < len(self.selected_path['waypoints']):
            if not self.is_waiting_for_robot:
                # 다음 웨이포인트 퍼블리시
                wp = self.selected_path['waypoints'][self.current_waypoint_index]
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose.position.x = wp['pose'][0]
                pose.pose.position.y = wp['pose'][1]
                pose.pose.position.z = 0.0
                pose.pose.orientation.w = 1.0  # 방향은 기본값으로 설정

                self.goal_pose_pub.publish(pose)
                self.get_logger().info(f"{self.current_waypoint_index+1}/{len(self.selected_path['waypoints'])}번째 웨이포인트 퍼블리시: {self.selected_path['waypoints'][self.current_waypoint_index]['pose']}")

                # Stop flag 퍼블리시
                stop_msg = Bool()
                stop_msg.data = wp.get('stop', False)
                self.goal_stop_pub.publish(stop_msg)
                self.get_logger().info(f"Stop flag: {stop_msg.data}")

                self.current_goal_pose = pose
                self.is_waiting_for_robot = True
            else:
                # 이미 목표 지점을 보낸 상태에서 추가 처리는 필요 없음
                pass
        else:
            self.get_logger().info("모든 웨이포인트 퍼블리시 완료.")
            self.all_waypoints_published = True
            self.timer.cancel()

    def goal_reached_callback(self, msg):
        # 로봇이 목표 지점에 도착했음을 알림
        if self.is_waiting_for_robot and self.current_goal_pose is not None:
            # 현재 목표 지점과 로봇이 보낸 목표 지점이 동일한지 확인
            if self.is_same_pose(msg.pose, self.current_goal_pose.pose):
                self.get_logger().info("로봇이 목표 지점에 도착했습니다.")
                self.current_waypoint_index += 1
                self.is_waiting_for_robot = False
            else:
                # 동일하지 않은 목표 지점이면 무시
                self.get_logger().info("받은 목표 지점이 현재 목표와 다릅니다. 무시합니다.")

    def is_same_pose(self, pose1, pose2, tolerance=0.01):
        dx = pose1.position.x - pose2.position.x
        dy = pose1.position.y - pose2.position.y
        distance = math.sqrt(dx * dx + dy * dy)
        return distance < tolerance

    def zone_selection_callback(self, msg):
        zone = msg.data
        self.get_logger().info(f"수신된 구역 선택 값: {zone}")

        if zone in self.paths:
            # 새로운 경로 선택
            self.selected_path = self.paths[zone]
            self.get_logger().info(f"선택된 새로운 경로: {self.selected_path['name']}")

            # 웨이포인트 인덱스와 상태 초기화
            self.current_waypoint_index = 0
            self.is_waiting_for_robot = False
            self.current_goal_pose = None
            self.all_waypoints_published = False
            self.get_logger().info("웨이포인트 퍼블리싱 상태 초기화.")

            # 기존 타이머가 있다면 취소
            if self.timer is not None:
                self.timer.cancel()

            # 웨이포인트 퍼블리싱 타이머 시작 (초당 2번, 0.5초 간격)
            self.timer = self.create_timer(0.5, self.timer_callback)
            self.get_logger().info("웨이포인트 퍼블리싱 타이머 시작.")

            # 카메라 퍼블리싱 타이머는 이제 필요 없으므로 제거
            # 이미 turtlebot4 서버에서 지속적으로 퍼블리시하고 있으므로
        else:
            self.get_logger().error(f"유효하지 않은 구역 선택 값: {zone}")

    def obstacle_stop_callback(self, msg):
        received_message = msg.data
        self.get_logger().info(f"obstacle_stop에서 수신된 메시지: {received_message}")
        
        if received_message == "고라니":
            # 장애물 정지 퍼블리시
            stop_flag = Bool()
            stop_flag.data = True
            self.obstacle_stop_flag_pub.publish(stop_flag)
            self.get_logger().warn("고라니 감지! waypoint_follower에게 장애물 정지 신호를 보냅니다.")
            
            # 목표 퍼블리싱 타이머 중단
            if self.timer is not None:
                self.timer.cancel()
                self.timer = None
                self.is_stopped = True
                self.get_logger().info("목표 퍼블리싱 타이머 중단.")
        else:
            # "고라니" 외의 메시지를 수신했을 때 장애물 정지 해제
            stop_flag = Bool()
            stop_flag.data = False
            self.obstacle_stop_flag_pub.publish(stop_flag)
            self.get_logger().info("장애물 정지 신호 해제.")

            # 정지 상태였다면 퍼블리싱 재개
            if self.is_stopped:
                self.is_stopped = False
                self.timer = self.create_timer(0.5, self.timer_callback)
                self.get_logger().info("목표 퍼블리싱 타이머 재개.")

    def run(self):
        # 노드 스핀
        rclpy.spin(self)

def main(args=None):
    rclpy.init(args=args)

    # 노드 생성
    node = WaypointPublisher()

    try:
        # 노드 실행
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("노드가 키보드 인터럽트로 종료됩니다.")
    finally:
        # 노드 종료
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
