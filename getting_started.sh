#!/bin/bash

# Install ROS 2 Humble
sudo apt update
sudo apt install -y curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu focal main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo apt update
sudo apt install -y ros-humble-desktop

# Source ROS 2 setup.bash
source /opt/ros/humble/setup.bash

# Create ROS 2 workspace
mkdir -p ~/ros2ws/src
cd ~/ros2ws

# Clone the package
git clone https://github.com/CDEI-Agro/agri_bot src/agri_bot

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build the package
colcon build

# Source the workspace
source install/setup.bash
