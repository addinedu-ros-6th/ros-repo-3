cmake_minimum_required(VERSION 3.8)
project(pollination_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)       # For C++ nodes
find_package(rclpy REQUIRED)        # For Python nodes
find_package(std_msgs REQUIRED)     # Replace with your message dependencies

# Add C++ executable
# add_executable(depthcam_pub src/depthcam_pub.cpp)
# ament_target_dependencies(depthcam_pub rclcpp std_msgs)

add_executable(pointcloud_generator src/pointcloud_generator.cpp)
target_link_libraries(pointcloud_generator realsense2 boost_system ${OpenCV_LIBRARIES})


# Install C++ node
install(TARGETS
  # depthcam_pub
  pointcloud_generator
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python nodes
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  src/raw_ebimu_pub.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  src/frame_generator.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
