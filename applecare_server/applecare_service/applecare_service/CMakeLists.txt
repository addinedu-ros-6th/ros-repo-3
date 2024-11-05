cmake_minimum_required(VERSION 3.5)
project(turtlebot4_custom_controller)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

# Include directories (if needed)
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# Install Python scripts (파이썬 파일을 설치할 때 이 코드를 사용합니다)
install(PROGRAMS
  scripts/controller_node.py
  scripts/waypoint_setter.py
  scripts/map_publisher.py
  scripts/applecare_publisher.py
  scripts/applecare_subscriber.py

  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# Export dependencies
ament_export_dependencies(rclpy sensor_msgs geometry_msgs nav_msgs tf2_ros visualization_msgs std_msgs OpenCV yaml-cpp)

# ament package
ament_package()
