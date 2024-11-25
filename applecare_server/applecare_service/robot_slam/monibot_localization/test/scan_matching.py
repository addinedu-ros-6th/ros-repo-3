import math
import numpy as np
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from rclpy.qos import QoSProfile, QoSHistoryPolicy

class LaserToOccupancyGrid(Node):
    def __init__(self):
        super().__init__('scan_to_map_node')
        qos_profile = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=10,
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT)
        
        # Load parameters
        self.scan_topic = self.declare_parameter("scan_topic", "/scan").get_parameter_value().string_value
        self.map_topic = self.declare_parameter("map_topic", "/map").get_parameter_value().string_value
        self.map_resolution = self.declare_parameter("map_resolution", 0.05).get_parameter_value().double_value
        self.map_width = self.declare_parameter("map_width", 200).get_parameter_value().integer_value
        self.map_height = self.declare_parameter("map_height", 200).get_parameter_value().integer_value
        self.map_origin_x = self.declare_parameter("map_origin_x", -5.0).get_parameter_value().double_value
        self.map_origin_y = self.declare_parameter("map_origin_y", -5.0).get_parameter_value().double_value
        self.map_frame = self.declare_parameter("map_frame", "map").get_parameter_value().string_value

        # Subscriptions and Publishers
        self.subscription = self.create_subscription(LaserScan, self.scan_topic, self.laser_callback, qos_profile)
        self.publisher = self.create_publisher(OccupancyGrid, self.map_topic, 10)

        self.get_logger().info("LaserScanToOccupancyGridNode initialized with parameters.")


    def bresenham(self, x0, y0, x1, y1):
        """
        Bresenham's line algorithm to get points on a line between two grid cells.
        
        Args:
            x0, y0 (int): Start grid coordinates.
            x1, y1 (int): End grid coordinates.

        Returns:
            List[Tuple[int, int]]: List of points on the line.
        """
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        while True:
            points.append((int(x0), int(y0)))
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy

        return points


    def process_laserscan_to_occupancygrid(self, scan_msg, map_resolution=0.05, map_size=(200, 200), origin=(-5.0, -5.0)):
        """
        Converts a LaserScan message to an OccupancyGrid.
        """
        grid_width, grid_height = map_size
        grid = -1 * np.ones((grid_height, grid_width), dtype=np.int8)  # Initialize with unknown (-1)

        robot_x, robot_y = -origin[0] / map_resolution, -origin[1] / map_resolution  # Robot in grid coordinates

        for i, r in enumerate(scan_msg.ranges):
            if r <= scan_msg.range_min or r > scan_msg.range_max or r == 0.0:
                continue

            angle = scan_msg.angle_min + i * scan_msg.angle_increment
            world_x = r * np.cos(angle)
            world_y = r * np.sin(angle)

            grid_x = int((world_x - origin[0]) / map_resolution)
            grid_y = int((world_y - origin[1]) / map_resolution)

            # self.get_logger().info(f"Valid range: {r}, Cartesian: ({world_x}, {world_y})")

            if grid_x < 0 or grid_x >= grid_width or grid_y < 0 or grid_y >= grid_height:
                continue

            # Mark endpoint as occupied
            grid[grid_y, grid_x] = 100

            # Mark free cells along the line to this point
            line_points = self.bresenham(robot_x, robot_y, grid_x, grid_y)
            for free_x, free_y in line_points:
                if 0 <= free_x < grid_width and 0 <= free_y < grid_height and grid[free_y, free_x] == -1:
                    grid[free_y, free_x] = 0
                    # self.get_logger().info(f"Grid updated: Occupied ({grid_x}, {grid_y}), Free: {line_points}")


            occupancy_grid_msg = OccupancyGrid()
            occupancy_grid_msg.header = scan_msg.header
            occupancy_grid_msg.header.frame_id = "map"
            occupancy_grid_msg.info.resolution = map_resolution
            occupancy_grid_msg.info.width = grid_width
            occupancy_grid_msg.info.height = grid_height
            occupancy_grid_msg.info.origin.position.x = origin[0]
            occupancy_grid_msg.info.origin.position.y = origin[1]
            occupancy_grid_msg.info.origin.orientation.w = 1.0
            occupancy_grid_msg.data = grid.flatten().tolist()

        return occupancy_grid_msg


    def laser_callback(self, scan_msg):
        occupancy_grid = self.process_laserscan_to_occupancygrid(scan_msg)
        self.publisher.publish(occupancy_grid)
        self.get_logger().info("Published OccupancyGrid map.")

def main(args=None):
    rclpy.init(args=args)
    node = LaserToOccupancyGrid()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
