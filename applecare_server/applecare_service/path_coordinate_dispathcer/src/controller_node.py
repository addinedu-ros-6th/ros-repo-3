#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
import yaml
import math

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        
        # Parameters
        self.declare_parameter('waypoints_file', 'config/waypoints.yaml')
        waypoints_file = self.get_parameter('waypoints_file').get_parameter_value().string_value
        
        # Load waypoints
        self.waypoints = []
        self.current_waypoint_index = 0
        self.load_waypoints(waypoints_file)
        
        # Publishers and Subscribers
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marker_publisher = self.create_publisher(MarkerArray, 'waypoints', 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # Timer for control loop (10 Hz)
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        # Timer for publishing markers (1 Hz)
        self.marker_timer = self.create_timer(1.0, self.publish_markers)
        
        # Current position
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        
        self.get_logger().info('Controller Node Initialized.')

    def load_waypoints(self, file_path):
        try:
            with open(file_path, 'r') as f:
                data = yaml.safe_load(f)
                initial = data.get('initial_position', {})
                self.initial_x = initial.get('x', 0.0)
                self.initial_y = initial.get('y', 0.0)
                self.initial_theta = initial.get('theta', 0.0)
                
                waypoints = data.get('waypoints', [])
                for wp in waypoints:
                    self.waypoints.append((wp.get('x', 0.0), wp.get('y', 0.0)))
                
                end = data.get('end_position', {})
                self.end_x = end.get('x', 0.0)
                self.end_y = end.get('y', 0.0)
                self.end_theta = end.get('theta', 0.0)
                
                self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints.')
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints: {e}')

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Orientation
        orientation = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y ** 2 + orientation.z ** 2)
        self.current_theta = math.atan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        cmd = Twist()
        
        if self.current_waypoint_index < len(self.waypoints):
            target_x, target_y = self.waypoints[self.current_waypoint_index]
            
            dx = target_x - self.current_x
            dy = target_y - self.current_y
            distance = math.hypot(dx, dy)
            target_theta = math.atan2(dy, dx)
            angle_error = self.normalize_angle(target_theta - self.current_theta)
            
            distance_tolerance = 0.1  # meters
            angle_tolerance = 0.1     # radians
            
            if abs(angle_error) > angle_tolerance:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.5 * angle_error
            else:
                cmd.linear.x = 0.2
                cmd.angular.z = 0.0
            
            if distance < distance_tolerance:
                self.get_logger().info(f'Reached Waypoint {self.current_waypoint_index}')
                self.current_waypoint_index += 1
                if self.current_waypoint_index >= len(self.waypoints):
                    self.get_logger().info('All waypoints reached.')
        else:
            # All waypoints reached, stop
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        
        self.cmd_publisher.publish(cmd)

    def publish_markers(self):
        marker_array = MarkerArray()
        
        # Initial Position Marker (Blue)
        initial_marker = Marker()
        initial_marker.header.frame_id = 'map'
        initial_marker.header.stamp = self.get_clock().now().to_msg()
        initial_marker.ns = 'waypoints'
        initial_marker.id = -1
        initial_marker.type = Marker.SPHERE
        initial_marker.action = Marker.ADD
        initial_marker.pose.position.x = self.initial_x
        initial_marker.pose.position.y = self.initial_y
        initial_marker.pose.position.z = 0.0
        initial_marker.scale.x = 0.3
        initial_marker.scale.y = 0.3
        initial_marker.scale.z = 0.3
        initial_marker.color.a = 1.0
        initial_marker.color.b = 1.0
        marker_array.markers.append(initial_marker)
        
        # Waypoints Markers (Green)
        for idx, (x, y) in enumerate(self.waypoints):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'waypoints'
            marker.id = idx
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.0
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.g = 1.0
            marker_array.markers.append(marker)
        
        # End Position Marker (Red)
        end_marker = Marker()
        end_marker.header.frame_id = 'map'
        end_marker.header.stamp = self.get_clock().now().to_msg()
        end_marker.ns = 'waypoints'
        end_marker.id = len(self.waypoints)
        end_marker.type = Marker.SPHERE
        end_marker.action = Marker.ADD
        end_marker.pose.position.x = self.end_x
        end_marker.pose.position.y = self.end_y
        end_marker.pose.position.z = 0.0
        end_marker.scale.x = 0.3
        end_marker.scale.y = 0.3
        end_marker.scale.z = 0.3
        end_marker.color.a = 1.0
        end_marker.color.r = 1.0
        marker_array.markers.append(end_marker)
        
        self.marker_publisher.publish(marker_array)
        self.get_logger().info('Markers published.')

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
