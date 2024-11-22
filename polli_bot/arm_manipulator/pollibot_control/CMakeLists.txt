cmake_minimum_required(VERSION 3.8)
project(pollibot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(pollibot_moveit_config REQUIRED)
find_package(pollibot_arm_description REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(move_to_ready_pose src/move_to_ready_pose.cpp)
ament_target_dependencies(move_to_ready_pose 
  rclcpp
  moveit_ros_planning_interface 
  ament_index_cpp
)

install(TARGETS move_to_ready_pose
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS src/pollitask_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pollitask_node
)

add_executable(start_push src/start_push.cpp)
ament_target_dependencies(start_push
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  geometry_msgs
  tf2_geometry_msgs
)

install(TARGETS start_push
  DESTINATION lib/${PROJECT_NAME}
)

################################################################

add_executable(move_to_target src/move_to_target.cpp)
ament_target_dependencies(move_to_target
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  geometry_msgs
  tf2_geometry_msgs
)

install(TARGETS move_to_target
  DESTINATION lib/${PROJECT_NAME}
)

#################################################################

add_executable(search_IK_solver src/search_IK_solver.cpp)
ament_target_dependencies(search_IK_solver
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  geometry_msgs
  tf2_geometry_msgs
)

install(TARGETS search_IK_solver
  DESTINATION lib/${PROJECT_NAME}
)


# Install Python scripts
# install(
#   PROGRAMS src/pollitask_node.py src/end_effector_marker_publisher.py
#   DESTINATION lib/${PROJECT_NAME}
#   RENAME pollitask_launcher  # Keeps the Python node name consistent
# )

# install the launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
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
