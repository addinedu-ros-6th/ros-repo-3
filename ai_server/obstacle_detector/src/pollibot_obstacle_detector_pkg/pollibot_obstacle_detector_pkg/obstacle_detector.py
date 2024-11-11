import rclpy as rp
from rclpy.node import Node
from ultralytics import YOLO
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
import cv2
import numpy as np

class tree_status_detection_node(Node):
    def __init__(self):
        super().__init__('tree_status_detection_node')
        self.tree_camera_subscriber = self.create_subscription(CompressedImage, 'forwarded_tree_image', self.image_callback, 10)
        self.tree_camera_subscriber
        self.tree_status_detection_model = YOLO("/home/jh/project/final_project/ai_server/tree_status/src/tree_status_package/best.pt")
        self.detection_data = ""

        self.tree_status_detection_info = self.create_publisher(String, 'obstacle_stop', 10)
        self.tree_status_detection_info
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def image_callback(self, msg):
        image_array = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(image_array, cv2.IMREAD_COLOR)
        
        tree_status_detection_results = self.tree_status_detection_model(frame)
        annotated_frame = tree_status_detection_results[0].plot()

        tree_status_detections = tree_status_detection_results[0].boxes
        # self.detection_data = str(tree_status_detections)

        tree_status_detection_data = []
        for box in tree_status_detections:
            class_id = box.cls
            x1, y1, x2, y2 = map(str, box.xyxy[0])
            tree_status_detection_data.append((class_id, x1, y1, x2, y2))
        
        if len(tree_status_detection_data) > 0:
            self.detection_data = "고라니"
        else:
            self.detection_data = ""

        if frame is not None:
            cv2.imshow("Received Image", annotated_frame)
            cv2.waitKey(1)

    def timer_callback(self):
        msg = String()
        msg.data = self.detection_data

        # msg.data = "hello \n -------"
        if msg.data == "고라니":
            self.tree_status_detection_info.publish(msg=msg)

            self.get_logger().info(f"{msg.data}")

def main(args=None):
    rp.init(args=args)

    tree_status_detect = tree_status_detection_node()
    rp.spin(tree_status_detect)

    tree_status_detect.destroy_node()
    rp.shutdown

if __name__ == "__main__":
    main()