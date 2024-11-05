#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import yaml
import cv2
import os
from ament_index_python.packages import get_package_share_directory
import sys

class WaypointSetter(Node):
    def __init__(self):
        super().__init__('waypoint_setter')
        
        # Parameters
        self.declare_parameter('map_file', 'config/map.yaml')
        self.map_file = self.get_parameter('map_file').get_parameter_value().string_value
        
        # Load map
        self.load_map(self.map_file)
        
        # Initialize variables
        self.start_set = False
        self.end_set = False
        self.waypoints = []
        self.start_pos = {}
        self.end_pos = {}
        self.current_mode = None
        
        # Create OpenCV window and set mouse callback
        cv2.namedWindow('Map')
        cv2.setMouseCallback('Map', self.mouse_callback)
        
        self.get_logger().info('Waypoint Setter Node Initialized.')
        self.get_logger().info("Press 's' to set Start, 'p' to add Waypoint, 'e' to set End, 'q' to quit and save.")
        
        # Display the map
        self.display_map()
        
        # Spin in a separate thread
        self.create_timer(0.1, self.check_keypress)
    
    def load_map(self, file_path):
        try:
            with open(file_path, 'r') as f:
                data = yaml.safe_load(f)
                image_path = data.get('image', 'map.pgm')
                resolution = data.get('resolution', 0.05)
                origin = data.get('origin', [0.0, 0.0, 0.0])
                self.resolution = resolution
                self.origin = origin
                # Handle relative path
                if not os.path.isabs(image_path):
                    pkg_share = get_package_share_directory('applecare_service')
                    image_path = os.path.join(pkg_share, image_path)
                self.map_image = cv2.imread(image_path)
                if self.map_image is None:
                    self.get_logger().error(f'Failed to load map image: {image_path}')
                    rclpy.shutdown()
                    sys.exit(1)
        except Exception as e:
            self.get_logger().error(f'Failed to load map file: {e}')
            rclpy.shutdown()
            sys.exit(1)
    
    def display_map(self):
        self.display_image = self.map_image.copy()
        cv2.imshow('Map', self.display_image)
    
    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            if self.current_mode == 's':
                self.start_pos = {'x': self.pixel_to_world_x(x), 'y': self.pixel_to_world_y(y), 'theta': 0.0}
                self.start_set = True
                self.get_logger().info(f'Start position set at ({self.start_pos["x"]}, {self.start_pos["y"]})')
                cv2.circle(self.display_image, (x, y), 5, (255, 0, 0), -1)  # Blue
            elif self.current_mode == 'p':
                wp = {'x': self.pixel_to_world_x(x), 'y': self.pixel_to_world_y(y)}
                self.waypoints.append(wp)
                self.get_logger().info(f'Waypoint {len(self.waypoints)} set at ({wp["x"]}, {wp["y"]})')
                cv2.circle(self.display_image, (x, y), 5, (0, 255, 0), -1)  # Green
            elif self.current_mode == 'e':
                self.end_pos = {'x': self.pixel_to_world_x(x), 'y': self.pixel_to_world_y(y), 'theta': 0.0}
                self.end_set = True
                self.get_logger().info(f'End position set at ({self.end_pos["x"]}, {self.end_pos["y"]})')
                cv2.circle(self.display_image, (x, y), 5, (0, 0, 255), -1)  # Red
            else:
                self.get_logger().warn("Set mode first: 's', 'p', or 'e'")
            cv2.imshow('Map', self.display_image)
    
    def pixel_to_world_x(self, x_pixel):
        return self.origin[0] + (x_pixel * self.resolution)
    
    def pixel_to_world_y(self, y_pixel):
        # OpenCV y-axis increases downwards; ROS map y-axis increases upwards
        return self.origin[1] + ((self.map_image.shape[0] - y_pixel) * self.resolution)
    
    def check_keypress(self):
        key = cv2.waitKey(1) & 0xFF
        if key == ord('s'):
            self.current_mode = 's'
            self.get_logger().info("Set to Start position mode.")
        elif key == ord('p'):
            self.current_mode = 'p'
            self.get_logger().info("Set to Waypoint add mode.")
        elif key == ord('e'):
            self.current_mode = 'e'
            self.get_logger().info("Set to End position mode.")
        elif key == ord('q'):
            self.get_logger().info("Saving waypoints and quitting.")
            self.save_waypoints()
            cv2.destroyAllWindows()
            rclpy.shutdown()
            sys.exit(0)
    
    def save_waypoints(self):
        if not self.start_set:
            self.get_logger().error("Start position not set.")
            return
        if not self.end_set:
            self.get_logger().error("End position not set.")
            return
        
        waypoints_data = {
            'initial_position': self.start_pos,
            'waypoints': self.waypoints,
            'end_position': self.end_pos
        }
        
        try:
            pkg_share = get_package_share_directory('applecare_server')
            config_dir_install = os.path.join(pkg_share, 'config')
            config_dir_src = '/home/leesiwon/turtlebot4_ws/src/applecare_server/config'
            os.makedirs(config_dir_src, exist_ok=True)

            # Save in both install and source config folders
            waypoints_file_install = os.path.join(config_dir_install, 'waypoints.yaml')
            waypoints_file_src = os.path.join(config_dir_src, 'waypoints.yaml')

            with open(waypoints_file_install, 'w') as f_install:
                yaml.dump(waypoints_data, f_install)
            with open(waypoints_file_src, 'w') as f_src:
                yaml.dump(waypoints_data, f_src)
            
            self.get_logger().info(f'Waypoints saved to {waypoints_file_install} and {waypoints_file_src}')
        except Exception as e:
            self.get_logger().error(f'Failed to save waypoints: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = WaypointSetter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
