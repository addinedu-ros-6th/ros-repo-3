# Initialization of monibot_localization

1. Launch robot description

```
#monibot
ros2 launch minibot_bringup bringup.launch.py

#pollibot
ros2 launch turtlebot4_description robot_description.launch.py
```

2. Launch the ekf filter, map publisher, and amcl node

```
ros2 launch monibot_localization localization.launch.py
```

