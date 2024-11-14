# UDP code
import socket
import struct
import cv2
import numpy as np

# Define IP and Ports
UDP_IP = "192.168.0.5"  # Receiver IP address
# UDP_IP = "192.168.219.182"
RGB_PORT = 5555
DEPTH_PORT = 5333

# Define maximum buffer size for UDP packets
BUFFER_SIZE = 65507

# Initialize sockets for receiving RGB and depth data
sock_rgb = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_rgb.bind((UDP_IP, RGB_PORT))

sock_depth = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_depth.bind((UDP_IP, DEPTH_PORT))

# Function to receive and reassemble chunks
def receive_data(sock):
	data_chunks = {}
	total_chunks = None

	while True:
		# Receive packet
		packet, _ = sock.recvfrom(BUFFER_SIZE)
		
		# Unpack header for chunk index and total chunks
		chunk_idx, total_chunks = struct.unpack(">HH", packet[:4])
		data = packet[4:]  # Extract actual data

		# Store chunk in the dictionary
		data_chunks[chunk_idx] = data

		# Break if we have received all chunks
		if len(data_chunks) == total_chunks:
			# Reassemble the complete data in order
			complete_data = b''.join(data_chunks[i] for i in range(total_chunks))
			return complete_data

# Main loop for receiving and displaying RGB and depth frames
try:
	while True:
		# Receive complete color and depth frames
		color_data = receive_data(sock_rgb)
		depth_data = receive_data(sock_depth)

		# Decode JPEG data to images
		color_frame = cv2.imdecode(np.frombuffer(color_data, np.uint8), cv2.IMREAD_COLOR)
		depth_frame = cv2.imdecode(np.frombuffer(depth_data, np.uint8), cv2.IMREAD_COLOR)


		if color_frame is None:
			print("NO RGB FOUND!!!!!!!!!!!!!!!!!")
		else:
			print("RGB success")
		if depth_frame is None:
			print("NO depth FOUND!!!!!!!!!!!!!!!!!")
		else:
			print("depth success")



		# Display the frames
		cv2.imshow('Received Color Frame', color_frame)
		cv2.imshow('Received Depth Frame', depth_frame)

		# Exit on 'q' key press
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break

finally:
	# Close sockets and windows
	sock_rgb.close()
	sock_depth.close()
	cv2.destroyAllWindows()



# ---------------------------------------------------
# ROS2 code
# import rclpy
# from rclpy.node import Node
# from cv_bridge import CvBridge
# from sensor_msgs.msg import Image
# import cv2
# import numpy as np
# from rclpy.qos import QoSProfile, ReliabilityPolicy

# class IntelSubscriber(Node):
#     def __init__(self):
#         super().__init__("intel_subscriber")
#         qos_profile = QoSProfile(depth=10)
#         qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT

#         self.intel_subscription_rgb = self.create_subscription(
#             Image, "rgb_frame_chunked", self.rgb_frame_callback, qos_profile
#         )
#         self.intel_subscription_depth = self.create_subscription(
#             Image, "depth_frame_chunked", self.depth_frame_callback, qos_profile
#         )

#         self.br_rgb = CvBridge()
#         self.br_depth = CvBridge()
#         self.rgb_data = {}
#         self.depth_data = {}

#     def process_frame(self, data_dict, msg, frame_type):
#         total_size = msg.width
#         num_chunks = msg.step
#         chunk_index = msg.height
#         chunk_data = msg.data

#         if frame_type not in data_dict:
#             data_dict[frame_type] = [None] * num_chunks

#         data_dict[frame_type][chunk_index] = chunk_data

#         if all(data_dict[frame_type]):
#             full_data = b"".join(data_dict[frame_type])
#             image_np = np.frombuffer(full_data, dtype=np.uint8)
#             image = cv2.imdecode(image_np, cv2.IMREAD_COLOR)

#             if image is None:
#                 self.get_logger().error(f"Failed to decode {frame_type} image")
#             else:
#                 cv2.imshow(frame_type, image)
#                 cv2.waitKey(1)
#                 self.get_logger().info(f"Displayed complete {frame_type} image")
            
#             # Clear data after processing
#             data_dict[frame_type] = [None] * num_chunks

#     def rgb_frame_callback(self, msg):
#         self.process_frame(self.rgb_data, msg, "RGB")

#     def depth_frame_callback(self, msg):
#         self.process_frame(self.depth_data, msg, "Depth")

# def main(args=None):
#     rclpy.init(args=args)
#     intel_subscriber = IntelSubscriber()
#     rclpy.spin(intel_subscriber)
#     intel_subscriber.destroy_node()
#     rclpy.shutdown()



