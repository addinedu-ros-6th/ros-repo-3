#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from std_msgs.msg import Bool
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import time
import math

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')

        self.get_logger().info("웨이포인트 팔로워 초기화 중...")

        # 초기 위치 설정을 위한 토픽 구독
        self.initial_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.initial_pose_callback,
            10
        )

        # 목표 위치를 받기 위한 토픽 구독
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_pose_callback,
            10
        )

        # AMCL 위치 구독
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_pose_callback,
            10
        )

        # 목표 퍼블리셔 (Nav2로 목표를 보내기 위해)
        self.nav_goal_pub = self.create_publisher(
            PoseStamped,
            '/goal_pose_nav',
            10
        )  # Nav2와의 충돌 방지를 위해 토픽 이름 변경

        # 목표 지점 도착 여부를 알리기 위한 퍼블리셔
        self.goal_reached_pub = self.create_publisher(
            PoseStamped,
            '/goal_reached',
            10
        )

        # Stop flag 구독자 추가 (정기적인 정지)
        self.goal_stop_sub = self.create_subscription(
            Bool,
            '/goal_stop',
            self.goal_stop_callback,
            10
        )

        # 장애물 정지 flag 구독자 추가
        self.obstacle_stop_flag_sub = self.create_subscription(
            Bool,
            '/obstacle_stop_flag',
            self.obstacle_stop_flag_callback,
            10
        )

        # cmd_vel 퍼블리셔 추가
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        # CvBridge 생성
        self.cvbridge = CvBridge()

        # 카메라 퍼블리셔 변경: Image -> CompressedImage
        self.camera_forward_pub = self.create_publisher(
            CompressedImage,
            '/forwarded_tree_image',
            10
        )
        self.camera_sub = self.create_subscription(
            Image,
            'oakd/rgb/preview/image_raw',
            self.camera_callback,
            10
        )
        self.get_logger().info("카메라 구독자 초기화 완료.")

        # 이미지 저장 변수
        self.latest_image = None

        # 상태 변수 초기화
        self.current_pose = None
        self.initial_pose_received = False
        self.goal_pose_received = False
        self.goal_pose = None
        self.is_navigating = False
        self.goal_tolerance = 0.5  # 허용 오차 범위 설정 
        self.current_stop_flag = False  # 정지 여부 플래그
        self.obstacle_stopped = False  # 장애물 정지 여부 플래그

        # 추가된 변수들
        self.waiting_after_stop = False
        self.stop_wait_start_time = None

    def initial_pose_callback(self, msg):
        if not self.initial_pose_received:
            self.get_logger().info("초기 위치 수신 및 퍼블리시 완료.")
            self.initial_pose_received = True
            # 초기 위치 퍼블리시 (Nav2 라이프사이클 활성화를 위해)
            initial_pose_pub = self.create_publisher(
                PoseWithCovarianceStamped,
                '/initialpose_nav',
                10
            )  # Nav2와의 충돌 방지를 위해 토픽 이름 변경
            for _ in range(10):
                initial_pose_pub.publish(msg)
                time.sleep(0.1)  # rclpy.sleep 대신 time.sleep 사용

    def goal_pose_callback(self, msg):
        if self.current_stop_flag or self.obstacle_stopped:
            self.get_logger().warn("현재 정지 상태입니다. 새로운 목표를 무시합니다.")
            return

        self.get_logger().info("목표 위치 수신 완료.")
        self.goal_pose = msg
        self.goal_pose_received = True

        # 현재 stop flag 상태 저장
        stop_flag = self.current_stop_flag

        # Stop flag가 true이고 현재 위치가 수신된 경우, 목표의 오리엔테이션을 현재 로봇의 오리엔테이션으로 설정
        if stop_flag and self.current_pose is not None:
            self.get_logger().info("Stop flag가 활성화되어 있습니다. 목표의 오리엔테이션을 현재 오리엔테이션으로 설정합니다.")
            self.goal_pose.pose.orientation = self.current_pose.orientation

        # 현재 위치와 목표 위치의 거리를 계산
        if self.current_pose is not None:
            distance = self.calculate_distance(
                self.current_pose.position,
                self.goal_pose.pose.position
            )
            self.get_logger().info(f"목표까지의 거리: {distance:.2f}m")

            if distance < self.goal_tolerance:
                self.get_logger().info("목표 지점이 허용 오차 범위 내에 있습니다. 목표 도달 메시지를 메인 서버로 보냅니다.")
                # 목표 지점 도착 메시지 퍼블리시
                self.goal_reached_pub.publish(self.goal_pose)
                self.is_navigating = False
                return  # 목표 지점이므로 이동하지 않음

        self.send_goal(self.goal_pose)

    def goal_stop_callback(self, msg):
        if msg.data:
            if not self.obstacle_stopped:
                self.get_logger().info("정기적인 정지 신호 수신! 로봇을 5초간 정지시킵니다.")
                self.stop_robot()
                self.is_navigating = False  # 현재 네비게이션 상태를 중지로 설정

                # Set waiting_after_stop flag and record start time
                self.waiting_after_stop = True
                self.stop_wait_start_time = self.get_clock().now()
                self.get_logger().info("5초간 정지 시작...")
        else:
            self.get_logger().info("정기적인 정지 신호 해제.")

    def obstacle_stop_flag_callback(self, msg):
        if msg.data:
            self.get_logger().warn("장애물 정지 신호 수신! 로봇을 즉시 정지시킵니다.")
            self.stop_robot()
            self.is_navigating = False
            self.obstacle_stopped = True  # 장애물 정지 상태 플래그 설정
        else:
            self.get_logger().info("장애물 정지 신호 해제.")
            self.obstacle_stopped = False  # 장애물 정지 상태 플래그 해제

    def amcl_pose_callback(self, msg):
        self.current_pose = msg.pose.pose

    def send_goal(self, pose):
        self.nav_goal_pub.publish(pose)
        self.get_logger().info(f"목표 위치 퍼블리시: x={pose.pose.position.x}, y={pose.pose.position.y}")
        self.is_navigating = True

    def calculate_distance(self, pos1, pos2):
        dx = pos2.x - pos1.x
        dy = pos2.y - pos1.y
        return math.sqrt(dx * dx + dy * dy)

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            # 정기적인 정지 후 대기 처리
            if self.waiting_after_stop:
                now = self.get_clock().now()
                elapsed_time = (now - self.stop_wait_start_time).nanoseconds / 1e9
                if elapsed_time >= 5.0:
                    self.get_logger().info("5초간 정지 완료.")
                    if self.goal_pose is not None:
                        self.goal_reached_pub.publish(self.goal_pose)
                        self.get_logger().info("goal_reached 메시지 퍼블리시 완료.")
                    self.waiting_after_stop = False

            # 네비게이션 상태 처리
            if self.is_navigating and not self.obstacle_stopped:
                distance = self.distance_to_goal()
                if distance is not None:
                    self.get_logger().debug(f"목표까지의 거리: {distance:.2f}m")
                    if distance < self.goal_tolerance:
                        self.get_logger().info("목표 지점에 도착했습니다.")
                        if self.current_stop_flag:
                            self.get_logger().info("목표 지점에서 5초 동안 정지합니다.")
                            # Set waiting_after_stop flag and start timer
                            self.waiting_after_stop = True
                            self.stop_wait_start_time = self.get_clock().now()
                        # 목표 지점 도착 메시지 퍼블리시
                        self.goal_reached_pub.publish(self.goal_pose)
                        self.is_navigating = False

            time.sleep(0.1)

    def distance_to_goal(self):
        if self.current_pose is None or self.goal_pose is None:
            return None
        return self.calculate_distance(
            self.current_pose.position,
            self.goal_pose.pose.position
        )

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info("cmd_vel을 통해 로봇을 정지시킵니다.")

    def camera_callback(self, msg):
        # CvBridge를 사용하여 Image 메시지를 OpenCV 이미지로 변환
        try:
            cv_image = self.cvbridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CvBridge 변환 중 오류 발생: {e}")
            return

        # 압축된 이미지 메시지 생성
        try:
            compressed_msg = self.cvbridge.cv2_to_compressed_imgmsg(cv_image, dst_format="jpeg")
            self.camera_forward_pub.publish(compressed_msg)
            self.get_logger().debug("forwarded_tree_image (CompressedImage) 퍼블리시")
        except Exception as e:
            self.get_logger().error(f"CompressedImage 퍼블리시 중 오류 발생: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("노드가 키보드 인터럽트로 종료됩니다.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
