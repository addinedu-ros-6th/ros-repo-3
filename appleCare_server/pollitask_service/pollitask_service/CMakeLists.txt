cmake_minimum_required(VERSION 3.8)
project(pollitask_service)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

#add_executable(trajectory_planner src/trajectory_planner.cpp)
#ament_target_dependencies(trajectory_planner rclcpp std_msgs geometry_msgs)

#add_executable(3d_pose_estimator src/3d_pose_estimator.cpp)
#ament_target_dependencies(3d_pose_estimator rclcpp std_msgs geometry_msgs)

add_executable(pollitask_service src/pollitask_service.cpp)
ament_target_dependencies(pollitask_service rclcpp std_msgs)

#install(TARGETS
#  trajectory_planner
#  3d_pose_estimator
#  pollitask_service
#  DESTINATION lib/${PROJECT_NAME}
#)

install(TARGETS
  pollitask_service
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python nodes
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  src/ebimu_sub.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  src/depthcam_sub.py
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
