import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy

class IntelSubscriber(Node):
    def __init__(self):
        super().__init__("intel_subscriber")
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT

        self.intel_subscription_rgb = self.create_subscription(
            Image, "rgb_frame_chunked", self.rgb_frame_callback, qos_profile
        )
        self.intel_subscription_depth = self.create_subscription(
            Image, "depth_frame_chunked", self.depth_frame_callback, qos_profile
        )

        self.br_rgb = CvBridge()
        self.br_depth = CvBridge()
        self.rgb_data = {}
        self.depth_data = {}

    def process_frame(self, data_dict, msg, frame_type):
        total_size = msg.width
        num_chunks = msg.step
        chunk_index = msg.height
        chunk_data = msg.data

        if frame_type not in data_dict:
            data_dict[frame_type] = [None] * num_chunks

        data_dict[frame_type][chunk_index] = chunk_data

        if all(data_dict[frame_type]):
            full_data = b"".join(data_dict[frame_type])
            image_np = np.frombuffer(full_data, dtype=np.uint8)
            image = cv2.imdecode(image_np, cv2.IMREAD_COLOR)

            if image is None:
                self.get_logger().error(f"Failed to decode {frame_type} image")
            else:
                cv2.imshow(frame_type, image)
                cv2.waitKey(1)
                self.get_logger().info(f"Displayed complete {frame_type} image")
            
            # Clear data after processing
            data_dict[frame_type] = [None] * num_chunks

    def rgb_frame_callback(self, msg):
        self.process_frame(self.rgb_data, msg, "RGB")

    def depth_frame_callback(self, msg):
        self.process_frame(self.depth_data, msg, "Depth")

def main(args=None):
    rclpy.init(args=args)
    intel_subscriber = IntelSubscriber()
    rclpy.spin(intel_subscriber)
    intel_subscriber.destroy_node()
    rclpy.shutdown()



# # ---------------------------------------- original subscriber code --------------------------------------------
# import cv2
# import rclpy
# from rclpy.node import Node
# from cv_bridge import CvBridge
# from sensor_msgs.msg import Image
# from rclpy.executors import MultiThreadedExecutor
# from rclpy.qos import QoSProfile, ReliabilityPolicy

# class IntelSubscriber(Node):
# 	def __init__(self):
# 		super().__init__("intel_subscriber")
# 		qos_profile = QoSProfile(depth=10)
# 		qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT
# 		self.intel_subscription_rgb = self.create_subscription(Image, "rgb_frame", self.rgb_frame_callback, qos_profile)

# 		# self.intel_subscription_rgb = self.create_subscription(Image, "rgb_frame", self.rgb_frame_callback, 10)
# 		self.intel_subscription_depth = self.create_subscription(Image, "depth_frame", self.depth_frame_callback, qos_profile)

# 		self.br_rgb = CvBridge()
# 		self.br_depth = CvBridge()

# 	def rgb_frame_callback(self, data):
# 		self.get_logger().warning("Receiving RGB frame")
# 		rgb_current_frame = self.br_rgb.imgmsg_to_cv2(data)
# 		cv2.imshow("RGB", rgb_current_frame)
# 		cv2.waitKey(1)

# 	def depth_frame_callback(self, data):
# 		self.get_logger().warning("Receiving depth frame")
# 		current_frame = self.br_depth.imgmsg_to_cv2(data)
# 		depth_cm = cv2.applyColorMap(cv2.convertScaleAbs(current_frame, alpha=0.08), cv2.COLORMAP_JET)
# 		cv2.imshow("Depth", depth_cm)
# 		cv2.waitKey(1)
	 


# def main(args=None):
# 	rclpy.init(args=None)
# 	intel_subscriber = IntelSubscriber()
# 	rclpy.spin(intel_subscriber)
# 	intel_subscriber.destroy_node()
# 	rclpy.shutdown()

# # def main(args=None):
# #         rclpy.init(args=None)
# #         intel_subscriber = IntelSubscriber()
# #         executor = MultiThreadedExecutor()
# #         executor.add_node(intel_subscriber)
# #         executor.spin()
# #         intel_subscriber.destroy_node()
# #         rclpy.shutdown()