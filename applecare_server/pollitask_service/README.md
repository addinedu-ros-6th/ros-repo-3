## 3D_pose_estimator

- receives the raw sensor values and convert into ROS2 topic
- computes the sensor values into VIO slam via external package (probably VINO or ORB or smth)
- outputs the VIO-generated values into ROS2 sensor_msg


## trajectoy_planner

- uses moveit2 to path-plan and navigate the manipulator (robotic arm)
