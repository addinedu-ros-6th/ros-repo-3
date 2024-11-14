## using ROS2 launch file to launch all the sensors

```ros2 launch pollination_manager generate_sensors.launch.py```



## STEP-BY-STEP on how to execute depth cam and IMU without ROS2 launch file

***caution: for generating the sensor values, we will be using UDP or serial to receive the raw sensor values and then convert it into ROS2 topic message types.***

1. For publishing the ***pointcloud data via UDP***, just generate the cpp file by the following codes (make sure to check if the server's IP address is correct)


```g++ dev_ws/pointcloud_generator.cpp -o generator -lrealsense2 -lboost_system `pkg-config --cflags --libs opencv4```

```./generator```

2. For publishing the ***rgb frame and depth frame via UDP***, just generate the python file by the following code (make sure to check if the server's IP address is correct)

```python3 frame_generator.py```


3. For publishing the ***IMU sensors via UDP***, just generate the python file by the following code(make sure to check if the server's IP address is correct)

```python3 raw_imu.py```





