# Cartographer Installation
- Please install the cartographer-ros before building the "slam_cartographer"

1. Clone the following repositories:

- https://github.com/cartographer-project/cartographer_ros
- https://github.com/cartographer-project/cartographer

```
cd ~/slam_ws/src
git clone -b ros2 https://github.com/ros2/cartographer.git
git clone -b ros2 https://github.com/ros2/cartographer_ros.git
```

2. Install all dependencies

```
sudo apt-get install -y \
  cmake \
  ninja-build \
  stow \
  libboost-all-dev \
  libgoogle-glog-dev \
  libgflags-dev \
  libcairo2-dev \
  libpcl-dev \
  libprotobuf-dev \
  protobuf-compiler \
  libeigen3-dev \
  liblz4-dev \
  libwebp-dev \
  libopenexr-dev \
  libabsl-dev \
  libceres-dev \
  liblua5.3-dev \
  lua5.3 \
  libgmock-dev \
  libgtest-dev \
  python3-sphinx \
  libogg-dev \
  libflann-dev \
  libqt5opengl5-dev
```

3. Build the packages
```
cd ~/slam_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select cartographer
source ~/slam_ws/install/setup.bash
```

* If any error occurs, try the below method
```
sudo apt-get update 
sudo apt-get install ros-humble-cartographer ros-humble-cartographer-ros
```


# Reference
- refer to these websites for more information on building a cartographer map

- https://medium.com/@a2jinhee/installing-google-cartographer-b05d506896be

- https://google-cartographer-ros.readthedocs.io/en/latest/compilation.html

- https://www.youtube.com/watch?v=G_1OOrabhu4
