#!/bin/bash

# Install ROS 2 Humble
sudo apt update
sudo apt install -y curl

# First ensure that the Ubuntu Universe repository is enabled.
sudo apt install software-properties-common
sudo add-apt-repository universe

# Now add the ROS 2 GPG key with apt.
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Then add the repository to your sources list.
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update your apt repository caches after setting up the repositories.
sudo apt update
sudo apt upgrade

# Desktop Install (Recommended): ROS, RViz, demos, tutorials.
sudo apt install ros-humble-desktop

# Source ROS 2 setup.bash
source /opt/ros/humble/setup.bash

# Create ROS 2 workspace
mkdir -p ~/ws/src
cd ~/ws

# # Clone the package
# git clone https://github.com/CDEI-Agro/agri_bot src/agri_bot

# # Install dependencies
# rosdep install --from-paths src --ignore-src -r -y

# # Build the package
# colcon build

# # Source the workspace
# source install/setup.bash
