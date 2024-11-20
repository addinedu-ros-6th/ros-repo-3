import rclpy as rp
from rclpy.node import Node
from ultralytics import YOLO
from sensor_msgs.msg import CompressedImage
from ai_server_msgs.msg import TreeStateDetectedObject
import cv2
import numpy as np

class tree_status_detection_node(Node):
    def __init__(self):
        super().__init__('tree_status_detection_node')
        self.tree_camera_subscriber = self.create_subscription(CompressedImage, '/monibot/tree_camera', self.image_callback, 10)
        self.tree_camera_subscriber
        self.tree_status_detection_model = YOLO("/home/jh/project/final_project/ai_server/src/ai_server_pkg/model/tree_state.pt")
        self.detection_data = ""

        self.tree_status_detection_info = self.create_publisher(TreeStateDetectedObject, '/monibot/tree_info', 10)
        self.tree_status_detection_info

    def image_callback(self, msg):
        image_array = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(image_array, cv2.IMREAD_COLOR)
        tree_status_detection_info_msg = TreeStateDetectedObject()
        tree_status_detection_info_msg.spot_detect = False
        tree_status_detection_info_msg.wilting_detect = False
        
        if frame is not None:
            self.get_logger().info("get monibot tree state frame")

            tree_status_detection_results = self.tree_status_detection_model(frame, conf=0.25, iou=0.5, classes=[1, 2])
            annotated_frame = tree_status_detection_results[0].plot()

            filtered_tree_status_detection_data = [box for box in tree_status_detection_results[0].boxes if box.conf[0] > 0.5]
            
            for box in filtered_tree_status_detection_data:
                class_id = int(box.cls)
                if class_id == 1:
                    tree_status_detection_info_msg.spot_detect = True
                elif class_id == 2:
                    tree_status_detection_info_msg.wilting_detect = True

        self.tree_status_detection_info.publish(msg=tree_status_detection_info_msg)
        self.get_logger().info(f"send monibot tree state detection info {tree_status_detection_info_msg}")

        if frame is not None:
            # cv2.imshow("Tree state Received Image", annotated_frame)
            cv2.imshow("Tree state Received Image", frame)
            cv2.waitKey(1)

def main(args=None):
    rp.init(args=args)

    tree_status_detect = tree_status_detection_node()
    rp.spin(tree_status_detect)

    tree_status_detect.destroy_node()
    rp.shutdown

if __name__ == "__main__":
    main()