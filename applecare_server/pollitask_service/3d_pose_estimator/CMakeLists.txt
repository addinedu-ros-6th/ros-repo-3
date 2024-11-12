# cmake_policy(SET CMP0072 OLD)
set(OpenGL_GL_PREFERENCE "LEGACY") # for visualizing via GL

cmake_minimum_required(VERSION 3.8)
project(3d_pose_estimator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Install C++ nodes
# currently, dont need this as we'll be using UDP to send the pointcloud data
add_executable(pointcloud_processor src/pointcloud_processor.cpp)
ament_target_dependencies(pointcloud_processor rclcpp sensor_msgs)
target_link_libraries(pointcloud_processor OpenGL::GL GLUT::GLUT)

install(TARGETS
  pointcloud_processor
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python nodes

install(PROGRAMS
  src/raw_ebimu_sub.py
  DESTINATION lib/${PROJECT_NAME}
)

# currently, dont need this as we'll be using UDP to send the raw rgb and depth frames data
install(PROGRAMS
  src/frame_sub.py
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
