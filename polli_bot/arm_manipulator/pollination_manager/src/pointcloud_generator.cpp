// pointcloud_generator.cpp

// g++ dev_ws/pointcloud_generator.cpp -o generator -lrealsense2 -lboost_system `pkg-config --cflags --libs opencv4`
// ./generator

#include <librealsense2/rs.hpp>    // Include RealSense Cross Platform API
#include <iostream>
#include <boost/asio.hpp>           // For UDP communication
#include <vector>
#include <thread>                   // For optional retry delay

using boost::asio::ip::udp;

int main() {
    try {
        // Initialize RealSense components
        rs2::pointcloud pc;
        rs2::points points;
        rs2::pipeline pipe;
        
        // Configure resolution and frame rate
        rs2::config cfg;
        cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 30);  // 640x480 resolution, 30 FPS
        cfg.enable_stream(RS2_STREAM_INFRARED, 640, 480, RS2_FORMAT_Y8, 30);   // 640x480 resolution, 30 FPS
        cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);   // 640x480 resolution, 30 FPS
        
        bool connected = false;
        while (!connected) {
            try {
                pipe.start(cfg);
                connected = true;
            } catch (const rs2::error & e) {
                std::cerr << "RealSense error: " << e.what() << std::endl;
                std::cerr << "Retrying connection in 1 second..." << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        }
        
        // Start the pipeline with custom configurations
        //pipe.start(cfg);

        // Setup UDP socket
        boost::asio::io_context io_context;
        udp::socket socket(io_context, udp::endpoint(udp::v4(), 0));
        udp::endpoint receiver_endpoint(boost::asio::ip::address::from_string("192.168.0.32"), 8080);

        while (true) {
            // Wait for frames from the camera
            auto frames = pipe.wait_for_frames();
            auto color_frame = frames.get_color_frame();
            if (!color_frame) color_frame = frames.get_infrared_frame();

            // Map pointcloud to the color frame
            pc.map_to(color_frame);

            // Generate the pointcloud and texture mappings
            auto depth_frame = frames.get_depth_frame();
            points = pc.calculate(depth_frame);

            // Retrieve point cloud data and downsample
            const rs2::vertex* vertices = points.get_vertices();
            size_t point_count = points.size();
            
            std::vector<float> buffer;
            int sample_rate = 4;  // Adjust this to control how much data you send

            for (size_t i = 0; i < point_count; i += sample_rate) {
                buffer.push_back(vertices[i].x);
                buffer.push_back(vertices[i].y);
                buffer.push_back(vertices[i].z);
            }

            // Send data in smaller chunks
            const size_t max_chunk_size = 12;  // Maximum bytes per chunk
            for (size_t i = 0; i < buffer.size(); i += max_chunk_size / sizeof(float)) {
                size_t chunk_size = std::min(max_chunk_size / sizeof(float), buffer.size() - i);
                socket.send_to(boost::asio::buffer(buffer.data() + i, chunk_size * sizeof(float)), receiver_endpoint);
            }
        }
    }
    catch (const rs2::error & e) {
        std::cerr << "RealSense error: " << e.what() << std::endl;
        // Optionally, add a delay and retry mechanism here
        std::this_thread::sleep_for(std::chrono::seconds(1));  // Retry after 1 second
    }
    catch (const std::exception & e) {
        std::cerr << "Standard exception: " << e.what() << std::endl;
    }
    return 0;
}
