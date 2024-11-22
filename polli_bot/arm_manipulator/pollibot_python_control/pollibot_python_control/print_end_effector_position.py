#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped
import tf_transformations

class EndEffectorPositionPrinter(Node):
    def __init__(self):
        super().__init__('end_effector_position_printer')

        # tf 버퍼와 리스너 초기화
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 주기적으로 위치를 출력하기 위해 타이머 설정 (1초마다)
        self.timer = self.create_timer(1.0, self.print_position)

    def print_position(self):
        try:
            # 'base_link'에서 'endeffector'로의 변환을 조회합니다.
            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform(
                'base_link',       # 소스 프레임
                'endeffector',     # 대상 프레임 (URDF에서 확인된 프레임 이름)
                now,
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            # 위치 정보 출력
            position = transform.transform.translation
            self.get_logger().info(f"End effector position: x={position.x}, y={position.y}, z={position.z}")

            # 방향 정보 출력 (쿼터니언)
            orientation = transform.transform.rotation
            self.get_logger().info(f"End effector orientation: x={orientation.x}, y={orientation.y}, z={orientation.z}, w={orientation.w}")

            # 쿼터니언을 RPY로 변환하여 출력
            quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
            roll, pitch, yaw = tf_transformations.euler_from_quaternion(quaternion)
            self.get_logger().info(f"End effector orientation (RPY): roll={roll}, pitch={pitch}, yaw={yaw}")

        except tf2_ros.LookupException:
            self.get_logger().error('Transform not available between base_link and endeffector')
        except tf2_ros.ExtrapolationException:
            self.get_logger().error('Extrapolation error when looking up transform')
        except tf2_ros.TransformException as ex:
            self.get_logger().error(f'Failed to get transform: {ex}')

def main(args=None):
    rclpy.init(args=args)
    node = EndEffectorPositionPrinter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
