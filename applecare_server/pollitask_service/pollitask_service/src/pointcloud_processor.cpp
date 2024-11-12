// pointcloud_processor.cpp
// ros2 run pollitask_service pointcloud_processor

#include <boost/asio.hpp>                    // For UDP communication
#include <iostream>
#include <GL/glut.h>                         // For OpenGL rendering
#include <vector>
#include <rclcpp/rclcpp.hpp>                 // ROS 2 core library
#include <sensor_msgs/msg/point_cloud2.hpp>  // PointCloud2 message type
#include <sensor_msgs/point_cloud2_iterator.hpp>

using boost::asio::ip::udp;

std::vector<float> point_cloud_data;
std::vector<float> last_valid_point_cloud;  // Store the last valid point cloud for comparison

// ROS 2 Node class for PointCloud2 publisher
class PointCloudReceiver : public rclcpp::Node {
public:
    PointCloudReceiver()
        : Node("pointcloud_processor") {
        // Initialize the publisher
        pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pointcloud_data", 10);

        // Start the UDP receiving thread
        std::thread(&PointCloudReceiver::receive_pointcloud_data, this).detach();

        // Start the OpenGL display (optional. only for debugging purpose)
        //std::thread(&PointCloudReceiver::start_display, this).detach();
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;

void receive_pointcloud_data() {
    boost::asio::io_context io_context;
    udp::socket socket(io_context, udp::endpoint(udp::v4(), 8080));

    const size_t max_point_cloud_size = 100000;  // Limit for x, y, z values (adjust as needed)

    while (rclcpp::ok()) {
        std::vector<float> buffer(512 / sizeof(float));  // Adjust buffer size to match sender's max_chunk_size
        udp::endpoint sender_endpoint;

        try {
            // Attempt to receive data
            size_t len = socket.receive_from(boost::asio::buffer(buffer), sender_endpoint);

            // Validate received data
            if (len == 0 || len % 12 != 0) {  // Each point is 3 floats (12 bytes)
                std::cerr << "Received incomplete or invalid packet, skipping." << std::endl;
                continue;
            }

            // Check for all-zero data
            bool is_all_zero = std::all_of(buffer.begin(), buffer.end(), [](float val) { return val == 0; });
            if (is_all_zero) {
                std::cerr << "All-zero data received, skipping." << std::endl;
                continue;
            }

            // Clear old data if necessary to avoid overflow
            if (point_cloud_data.size() + len / sizeof(float) > max_point_cloud_size) {
                point_cloud_data.clear();
            }

            // Insert validated data
            point_cloud_data.insert(point_cloud_data.end(), buffer.begin(), buffer.begin() + len / sizeof(float));

            publish_pointcloud();  // Publish the point cloud data to ROS 2

        } catch (const std::exception& e) {
            std::cerr << "Error receiving data: " << e.what() << std::endl;
            point_cloud_data.clear();  // Clear corrupted data on error
        }
    }
}


void publish_pointcloud() {
    // Create PointCloud2 message
    auto msg = sensor_msgs::msg::PointCloud2();
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "map";

    // Calculate the total number of points
    size_t num_points = point_cloud_data.size() / 3;  // Each point has x, y, z

    // Set the width, height, and fields
    msg.width = num_points;
    msg.height = 1;  // Unordered point cloud (width * height = total points)
    msg.is_dense = false;
    msg.is_bigendian = false;

    // Define fields: x, y, z
    sensor_msgs::PointCloud2Modifier modifier(msg);
    modifier.setPointCloud2Fields(3,
        "x", 1, sensor_msgs::msg::PointField::FLOAT32,
        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "z", 1, sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(num_points);

    // Copy data to PointCloud2
    sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");

    for (size_t i = 0; i < point_cloud_data.size(); i += 3) {  // Loop through point_cloud_data in groups of 3
        *iter_x = point_cloud_data[i];
        *iter_y = point_cloud_data[i + 1];
        *iter_z = point_cloud_data[i + 2];
        ++iter_x; ++iter_y; ++iter_z;
    }

    // Publish the message
    pointcloud_publisher_->publish(msg);
}

    // -----------------------------------------------------------------------------------
    // display pointcloud via RVIZ2 or
    // via GL by uncommenting the following code:

    // void start_display() {
    //     // Initialize OpenGL
    //     int argc = 0;
    //     char *argv[1] = {(char *)""};
    //     glutInit(&argc, argv);
    //     glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    //     glutInitWindowSize(320, 240);  // Adjust to desired display size
    //     glutCreateWindow("RealSense Pointcloud Receiver");
    //     glEnable(GL_DEPTH_TEST);

    //     // Register display function
    //     glutDisplayFunc(display);
    //     glutMainLoop();
    // }

    // static void display() {
    //     glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    //     glLoadIdentity();
    //     glPointSize(2.0);
    //     glBegin(GL_POINTS);

    //     // Draw the received point cloud data
    //     for (size_t i = 0; i < point_cloud_data.size(); i += 3) {
    //         float x = point_cloud_data[i];
    //         float y = point_cloud_data[i + 1];
    //         float z = point_cloud_data[i + 2];
    //         glVertex3f(x, y, z);
    //     }

    //     glEnd();
    //     glutSwapBuffers();
    // }


};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudReceiver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


// ----------------------------------------------------------------------------
// simpler code for debugging purpose

// class PointCloudProcessor : public rclcpp::Node {
// public:
//     PointCloudProcessor()
//         : Node("pointcloud_processor") {
//         pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pointcloud_data", 10);
//         publish_pointcloud();  // Call this for testing only
//     }

// private:
//     rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;

//     void publish_pointcloud() {
//         // Create a dummy PointCloud2 message for testing
//         auto msg = sensor_msgs::msg::PointCloud2();
//         msg.header.stamp = this->get_clock()->now();
//         msg.header.frame_id = "map";
//         msg.width = 1;
//         msg.height = 1;
//         msg.is_dense = false;
//         msg.is_bigendian = false;

//         // Define fields: x, y, z
//         sensor_msgs::PointCloud2Modifier modifier(msg);
//         modifier.setPointCloud2Fields(3,
//             "x", 1, sensor_msgs::msg::PointField::FLOAT32,
//             "y", 1, sensor_msgs::msg::PointField::FLOAT32,
//             "z", 1, sensor_msgs::msg::PointField::FLOAT32);
//         modifier.resize(1);

//         // Set a single dummy point
//         sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
//         sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
//         sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");
//         *iter_x = 0.0;
//         *iter_y = 0.0;
//         *iter_z = 0.0;

//         pointcloud_publisher_->publish(msg);
//     }
// };

// int main(int argc, char **argv) {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<PointCloudProcessor>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }

