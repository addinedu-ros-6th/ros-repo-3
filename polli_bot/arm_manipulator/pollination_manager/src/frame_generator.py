# frame_generator.py
# RGB and depth frame generator

# python3 frame_generator.py

import pyrealsense2 as rs
import numpy as np
import cv2
import socket
import struct

# Initialize UDP socket
UDP_IP = "192.168.0.5" #change this IP address to match with the server's IP address

# Assume socket and frame handling are already set up
RGB_PORT = 5555
DEPTH_PORT = 5333

# RGB socket setup
sock_rgb = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_rgb.bind(('', RGB_PORT))

# Depth socket setup
sock_depth = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_depth.bind(('', DEPTH_PORT))


# Initialize the RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)  # RGB stream
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)   # Depth stream



profile = pipeline.start(config)

# only for debugging purpose if the device is abnormal
# sensors = profile.get_device().query_sensors()
# for sensor in sensors:
#     print(f"{sensor} frame queue_size:", sensor.get_option(rs.option.frames_queue_size))
#     sensor.set_option(rs.option.frames_queue_size, 2)
#     print(f"{sensor} new queue_size:", sensor.get_option(rs.option.frames_queue_size))

#     if sensor.get_info(rs.camera_info.name) == "RGB Camera":
#         sensor.set_option(rs.option.exposure, 166.0)
#         print(f"{sensor.get_info(rs.camera_info.name)} exposure set to:", sensor.get_option(rs.option.exposure))
#     elif sensor.get_info(rs.camera_info.name) == "Stereo Module":
#         sensor.set_option(rs.option.exposure, 2858.0)
#         print(f"{sensor.get_info(rs.camera_info.name)} exposure set to:", sensor.get_option(rs.option.exposure))


# Function to send data in chunks
def send_data_in_chunks(sock, data, ip, port, chunk_size=60000):
    total_chunks = (len(data) + chunk_size - 1) // chunk_size  # Calculate total chunks
    for i in range(total_chunks):
        # Get the chunk data
        chunk = data[i * chunk_size:(i + 1) * chunk_size]
        # Send the chunk number, total chunks, and the chunk data
        sock.sendto(struct.pack(">HH", i, total_chunks) + chunk, (ip, port))

# Replace your sending code with this:
try:
    while True:
        # Wait for frames
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        if not color_frame or not depth_frame:
            continue

        # Convert RealSense frames to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        depth_image_8bit = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.08), cv2.COLORMAP_JET)
        #depth_image_8bit = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.08), cv2.IMREAD_GRAYSCALE)

        # Resize and encode depth image as JPEG
        resized_depth = cv2.resize(depth_image_8bit, (320, 240))
        resized_color = cv2.resize(color_image, (320, 240))

        _, jpeg_depth = cv2.imencode('.jpg', resized_depth)
        _, jpeg_color = cv2.imencode('.jpg', resized_color)

        # _, jpeg_depth = cv2.imencode('.jpg', depth_image_8bit)
        # _, jpeg_color = cv2.imencode('.jpg', color_image)

        depth_data = jpeg_depth.tobytes()
        depth_color = jpeg_color.tobytes()

        # Send the depth data in chunks to avoid "Message too long" error
        send_data_in_chunks(sock_rgb, depth_data, UDP_IP, RGB_PORT)
        send_data_in_chunks(sock_depth, depth_color, UDP_IP, DEPTH_PORT)

        # Display frames locally (optional only for debugging purpose)
        # cv2.imshow('Depth Frame', resized_depth)
        # cv2.imshow('RGB Frame', resized_color)
        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     break


finally:
    # Stop streaming and close resources
    pipeline.stop()
    sock_rgb.close()
    sock_depth.close()
    cv2.destroyAllWindows()