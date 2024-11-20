import rclpy as rp
from rclpy.node import Node
from ultralytics import YOLO
from sensor_msgs.msg import CompressedImage
from ai_server_msgs.msg import ObstacleDetectedObject, ObstacleDetectedObjects
import cv2
import numpy as np

class obstacle_detection_node(Node):
    def __init__(self):
        super().__init__('obstacle_detection_node')
        self.monibot_obstacle_camera_subscriber = self.create_subscription(CompressedImage, '/monibot/obstacle_camera', self.monibot_image_callback, 10)
        self.monibot_obstacle_camera_subscriber
        self.pollibot_obstacle_camera_subscriber = self.create_subscription(CompressedImage, '/pollibot/obstacle_camera', self.pollibot_image_callback, 10)
        self.pollibot_obstacle_camera_subscriber
        self.obstacle_detection_model = YOLO("/home/jh/project/final_project/ai_server/src/ai_server_pkg/model/obstacle.pt")

        self.monibot_obstacle_detection_info = self.create_publisher(ObstacleDetectedObjects, '/monibot/obstacle_info', 10)
        self.monibot_obstacle_detection_info
        self.pollibot_obstacle_detection_info = self.create_publisher(ObstacleDetectedObjects, '/pollibot/obstacle_info', 10)
        self.pollibot_obstacle_detection_info

    def monibot_image_callback(self, msg):
        frame = None
        image_array = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(image_array, cv2.IMREAD_COLOR)
        obstacle_detection_info_msg = ObstacleDetectedObjects()
        obstacle_detection_info_msg.count = 0

        if frame is not None:
            self.get_logger().info("get monibot obstacle frame")
            obstacle_detection_info_msg, annotated_frame = self.obstacle_detection(frame)

        self.monibot_obstacle_detection_info.publish(msg=obstacle_detection_info_msg)
        self.get_logger().info(f"send monibot obstacle detection info {obstacle_detection_info_msg}")

        if frame is not None:
            cv2.imshow("Monibot Received Image", annotated_frame)
            cv2.waitKey(1)
    
    def pollibot_image_callback(self, msg):
        frame = None
        image_array = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(image_array, cv2.IMREAD_COLOR)
        obstacle_detection_info_msg = ObstacleDetectedObjects()
        obstacle_detection_info_msg.count = 0

        if frame is not None:
            self.get_logger().info("get pollibot obstacle frame")
            obstacle_detection_info_msg, annotated_frame = self.obstacle_detection(frame)

        self.pollibot_obstacle_detection_info.publish(msg=obstacle_detection_info_msg)
        self.get_logger().info(f"send pollibot obstacle detection info {obstacle_detection_info_msg}")

        if frame is not None:
            # cv2.imshow("Pollibot Received Image", annotated_frame)
            cv2.imshow("Pollibot Received Image", frame)
            cv2.waitKey(1)

    def obstacle_detection(self, frame):
        obstacle_detection_info_msg = ObstacleDetectedObjects()
        obstacle_detection_info_msg.count = 0
    
        obstacle_detection_results = self.obstacle_detection_model(frame, conf=0.25, iou=0.5)
        annotated_frame = obstacle_detection_results[0].plot()
        
        sorted_obstacle_detection_data = sorted(obstacle_detection_results[0].boxes, key=lambda box: box.xyxy[0][0])
        filtered_obstacle_detection_data = [box for box in sorted_obstacle_detection_data if box.conf[0] > 0.5]

        if len(filtered_obstacle_detection_data) > 0:
            obstacle_detection_info_msg.count = len(filtered_obstacle_detection_data)
            
            for box in filtered_obstacle_detection_data:
                obj = ObstacleDetectedObject()
                obj.class_id = int(box.cls)
                obstacle_detection_info_msg.objects.append(obj)
        
        return obstacle_detection_info_msg, annotated_frame

def main(args=None):
    rp.init(args=args)

    obstacle_detect = obstacle_detection_node()
    rp.spin(obstacle_detect)

    obstacle_detect.destroy_node()
    rp.shutdown

if __name__ == "__main__":
    main()