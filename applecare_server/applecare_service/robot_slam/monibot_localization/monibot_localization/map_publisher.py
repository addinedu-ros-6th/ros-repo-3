import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from std_msgs.msg import Header
from geometry_msgs.msg import Pose
import yaml
import numpy as np
import cv2


class MapPublisher(Node):
    def __init__(self):
        super().__init__('map_publisher')
        # Static transform broadcaster
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_transform()

        # -------------- Publish /map -----------------------
        self.declare_parameter('frame_id', 'map')
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        self.declare_parameter('map_yaml_file', '/home/ksm/ws/ROS2/finalproject/dev_ros2/src/monibot_localization/map/cartographer_final_modified.yaml')
        map_yaml_file = self.get_parameter('map_yaml_file').get_parameter_value().string_value
        self.map_data = self.load_map(map_yaml_file)

        self.publisher_ = self.create_publisher(OccupancyGrid, 'map', 10)
        self.timer = self.create_timer(1.0, self.publish_map)  # Publish 2 second 0.5 Hz

        # -------------- Publish  /map_update ----------------
        # self.declare_parameter('update_interval', 2.0)
        # update_interval = self.get_parameter('update_interval').get_parameter_value().double_value

        # self.update_publisher = self.create_publisher(OccupancyGridUpdate, 'map_updates', 10)
        # self.update_timer = self.create_timer(update_interval, self.publish_map_update)  # Start timer to periodically publish map updates

    def load_map(self, yaml_file):
        """Load the map file and process it into OccupancyGrid format."""
        import os

        # Load the map metadata from the YAML file
        with open(yaml_file, 'r') as f:
            map_metadata = yaml.safe_load(f)

        image_path = map_metadata['image']
        if not image_path.startswith('/'):
            # Handle relative paths by appending to the directory of the YAML file
            image_path = os.path.join(os.path.dirname(yaml_file), image_path)

        # Load the map image
        image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise FileNotFoundError(f"Failed to load map image at {image_path}")

        resolution = map_metadata['resolution']
        origin = map_metadata['origin']

        # Flip the image vertically to match ROS map conventions
        image = cv2.flip(image, 0)

        # Explicitly map pixel values to occupancy grid values
        occupancy_grid = np.zeros_like(image, dtype=np.int8)
        occupancy_grid[image == 0] = 100   # Occupied
        occupancy_grid[image == 254] = 0  # Free
        occupancy_grid[image == 205] = -1 # Unknown -1 or 100

        return {
            'grid': occupancy_grid.flatten(),  # Flatten the 2D grid into 1D
            'width': image.shape[1],
            'height': image.shape[0],
            'resolution': resolution,
            'origin': origin,
            'frame_id': self.frame_id
        }


    def publish_map(self):
        map_msg = OccupancyGrid()
        map_msg.header = Header()
        map_msg.header.frame_id = self.frame_id
        map_msg.header.stamp = self.get_clock().now().to_msg()

        map_msg.info.resolution = self.map_data['resolution']
        map_msg.info.width = self.map_data['width']
        map_msg.info.height = self.map_data['height']
        map_msg.info.origin.position.x = self.map_data['origin'][0]
        map_msg.info.origin.position.y = self.map_data['origin'][1]
        map_msg.info.origin.position.z = 0.0
        map_msg.info.origin.orientation.w = 1.0

        map_msg.data = self.map_data['grid'].tolist()

        self.publisher_.publish(map_msg)

    def publish_map_update(self):
            """Publish an incremental map update to the /map_updates topic."""
            update_msg = OccupancyGridUpdate()
            update_msg.header = Header()
            update_msg.header.stamp = self.get_clock().now().to_msg()
            update_msg.header.frame_id = self.frame_id

            # Define the update region
            update_msg.x = 50
            update_msg.y = 50
            update_msg.width = 10
            update_msg.height = 10

            # Simulate an update (e.g., marking a 10x10 area as occupied)
            update_data = np.full((update_msg.width * update_msg.height,), 100, dtype=np.int8)
            update_msg.data = update_data.tolist()

            self.update_publisher.publish(update_msg)
            self.get_logger().info(f'Published map update to /map_updates at region ({update_msg.x}, {update_msg.y})')

    def publish_static_transform(self):
        # Create a static transform from "map" to "odom"
        static_transform = TransformStamped()
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = "map"
        static_transform.child_frame_id = "odom"

        static_transform.transform.translation.x = 0.0
        static_transform.transform.translation.y = 0.0
        static_transform.transform.translation.z = 0.0

        static_transform.transform.rotation.x = 0.0
        static_transform.transform.rotation.y = 0.0
        static_transform.transform.rotation.z = 0.0
        static_transform.transform.rotation.w = 1.0

        self.static_broadcaster.sendTransform(static_transform)
        self.get_logger().info('Published static transform: map -> odom')


def main(args=None):
    rclpy.init(args=args)
    node = MapPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
