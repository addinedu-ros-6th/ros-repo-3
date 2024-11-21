## 3D_pose_estimator

- receives the raw sensor values and convert into ROS2 topic
- computes the sensor values into VIO slam via external package (probably VINO or ORB or smth)
- outputs the VIO-generated values into ROS2 sensor_msg


## trajectoy_planner

- uses moveit2 to path-plan and navigate the manipulator (robotic arm)

## how to calculate flower pose

publish aligned image and depth data from realsense depth camera d435
- ros2 launch realsense2_camera rs_align_depth_launch.py

publish urdf of camera 
- ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro camera.urdf.xacro)"


calculate rgbd_odometry via rgb, depth (visual localization)
- ros2 run rtabmap_odom rgbd_odometry   --ros-args   --remap /depth/image:=/camera/camera/aligned_depth_to_color/image_raw   --remap /rgb/camera_info:=/camera/camera/color/camera_info   --remap /rgb/image:=/camera/camera/color/image_raw

in order to calculate the exact pose of flower, you need to get 2 different depth of flower via depth camera from different place. 
then calculate the real differences between camera and flower by triangular calculation.
you can calculate coordinate of flower from camera
then create flower's pose with camera's odometry.

