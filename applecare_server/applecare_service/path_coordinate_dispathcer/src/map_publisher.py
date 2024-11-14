#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import yaml
import cv2
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory
import sys

class MapPublisher(Node):
    def __init__(self):
        super().__init__('map_publisher')
        self.declare_parameter('map_file', 'config/map.yaml')
        map_file = self.get_parameter('map_file').get_parameter_value().string_value
        self.map_data = self.load_map(map_file)
        self.publisher_ = self.create_publisher(OccupancyGrid, 'map', 10)
        self.timer = self.create_timer(1.0, self.publish_map)
        self.get_logger().info('Map Publisher Initialized.')

    def load_map(self, file_path):
        try:
            with open(file_path, 'r') as f:
                data = yaml.safe_load(f)
            image_path = data['image']
            resolution = data['resolution']
            origin = data['origin']
            negate = data.get('negate', 0)
            occupied_thresh = data.get('occupied_thresh', 0.65)
            free_thresh = data.get('free_thresh', 0.196)

            # Handle relative path
            if not os.path.isabs(image_path):
                pkg_share = get_package_share_directory('applecare_service')
                image_path = os.path.join(pkg_share, image_path)

            map_image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
            if map_image is None:
                self.get_logger().error(f'Failed to load map image: {image_path}')
                rclpy.shutdown()
                sys.exit(1)
            
            # Apply thresholds
            occupancy = np.zeros(map_image.shape, dtype=np.int8)
            occupancy[map_image < free_thresh * 255] = 0
            occupancy[(map_image >= free_thresh * 255) & (map_image < occupied_thresh * 255)] = -1
            occupancy[map_image >= occupied_thresh * 255] = 100

            return {
                'width': map_image.shape[1],
                'height': map_image.shape[0],
                'resolution': resolution,
                'origin': origin,
                'data': occupancy.flatten().tolist()
            }
        except Exception as e:
            self.get_logger().error(f'Failed to load map file: {e}')
            rclpy.shutdown()
            sys.exit(1)

    def publish_map(self):
        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = 'map'
        map_msg.info.resolution = self.map_data['resolution']
        map_msg.info.width = self.map_data['width']
        map_msg.info.height = self.map_data['height']
        map_msg.info.origin.position.x = float(self.map_data['origin'][0])
        map_msg.info.origin.position.y = float(self.map_data['origin'][1])
        map_msg.info.origin.position.z = float(self.map_data['origin'][2]) 
        map_msg.info.origin.orientation.x = 0.0
        map_msg.info.origin.orientation.y = 0.0
        map_msg.info.origin.orientation.z = 0.0
        map_msg.info.origin.orientation.w = 1.0
        map_msg.data = self.map_data['data']
        self.publisher_.publish(map_msg)
        self.get_logger().info('Map published.')
        # 타이머 취소
        self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = MapPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
