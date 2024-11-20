import rclpy as rp
from rclpy.node import Node
from ultralytics import YOLO
from sensor_msgs.msg import CompressedImage
from ai_server_msgs.msg import FlowerDetectedObject, FlowerDetectedObjects
import cv2
import numpy as np

class pollination_detection_node(Node):
    def __init__(self):
        super().__init__('pollination_detection_node')
        self.flower_camera_subscriber = self.create_subscription(CompressedImage, '/pollibot/flower_camera', self.image_callback, 10)
        self.flower_camera_subscriber
        self.pollination_detection_model = YOLO("/home/jh/project/final_project/ai_server/src/ai_server_pkg/model/pollination.pt")
        # self.flower_direction_detection_model = YOLO("/home/jh/project/final_project/ai_server/src/ai_server_pkg/model/flower_direction.pt")

        self.flower_detection_info = self.create_publisher(FlowerDetectedObjects, '/pollibot/flower_detection_info', 10)
        self.flower_detection_info
        # self.timer_period = 0.1
        # self.timer = self.create_timer(self.timer_period, self.timer_callback)

    # 핀 꽃 박스 안에 암술, 수술이 인식되면 메세지 전송
    def image_callback(self, msg):
        
        # 카메라 수신
        image_array = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(image_array, cv2.IMREAD_COLOR)

        # 전송 메세지 초기화
        flower_detection_info_msg = FlowerDetectedObjects()
        flower_detection_info_msg.count = 0

        if frame is not None:
            # 프레임 수신 확인 로그
            self.get_logger().info("get pollibot flower frame")

            # 프레임 모델 검출 confidence 0.25(기본값)이상, iou(겹치는 박스) 0.6, classes(검출할 class_id 목록) 0 : center, 2 : blossom
            pollination_detection_results = self.pollination_detection_model(frame, conf=0.25, iou=0.6, classes=[0, 2])

            # 검출 결과 프레임
            pollination_annotated_frame = pollination_detection_results[0].plot()
            # 검출 결과 박스 정보
            pollination_detections = pollination_detection_results[0].boxes

            # 꽃의 방향 검출
            # flower_direction_detection_results = self.flower_direction_detection_model(frame, conf=0.25, iou=0.6, classes=[2])
            # flower_direction_annotated_frame = flower_direction_detection_results[0].plot()
            # flower_direciton_detections = flower_direction_detection_results[0].boxes

            # 꽃의 방향 정보(중심 좌표만) 저장
            # flower_front_detection_data = []
            # for box in flower_direciton_detections:
            #     x1, y1, x2, y2 = box.xyxy[0].tolist()
            #     flower_front_detection_data.append(((x1 + x2) / 2, (y1 + y2) / 2))

            # 검출 결과 중 핀 꽃에 대한 x1, y1, x2, y2 정보 저장
            blossom_detection_data = []
            for box in pollination_detections:
                class_id = int(box.cls)
                if class_id == 2:
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    blossom_detection_data.append((x1, y1, x2, y2))
            
            # 검출 결과 중 center가 인식된 중심 좌표가 핀 꽃이 인식된 좌표 안에 들어가 있으면 토픽 전송
            center_detection_data = []
            for box in pollination_detections:
                class_id = int(box.cls)
                # center class_id만 확인
                if class_id == 0:
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    
                    # 핀 꽃의 표 가져오기
                    for blossom_x1, blossom_y1, blossom_x2, blossom_y2 in blossom_detection_data:
                        # center의 중심 좌표 계산
                        center_x = (x1 + x2) / 2
                        center_y = (y1 + y2) / 2
                        # center의 중심 좌표가 blossom의 좌표 안에 있다면 object메세지 저장
                        if center_x > blossom_x1 and center_x < blossom_x2 and center_y > blossom_y1 and center_y < blossom_y2:
                            obj = FlowerDetectedObject()
                            obj.class_id = class_id
                            obj.x1 = x1
                            obj.y1 = y1
                            obj.x2 = x2
                            obj.y2 = y2
                            # 전송할 Objects 메세지 리스트에 추가
                            flower_detection_info_msg.objects.append(obj)
                            # 필터링한 중심 좌표 저장
                            center_detection_data.append((class_id, x1, y1, x2, y2))
            
            # 수분할 수 있는 중심 좌표 수 메세지 저장
            flower_detection_info_msg.count = len(center_detection_data)
        
        # 토픽 메세지 발송
        self.flower_detection_info.publish(msg=flower_detection_info_msg)
        self.get_logger().info(f"send pollibot flower detection info {flower_detection_info_msg}")

        if frame is not None:
            # cv2.imshow("pollination Received Image", pollination_annotated_frame)
            cv2.imshow("pollination Received Image", frame)
            cv2.waitKey(1)

def main(args=None):
    rp.init(args=args)

    tree_status_detect = pollination_detection_node()
    rp.spin(tree_status_detect)

    tree_status_detect.destroy_node()
    rp.shutdown

if __name__ == "__main__":
    main()