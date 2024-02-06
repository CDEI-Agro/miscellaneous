#!/bin/bash

#### Update and upgrade:
sudo apt update
sudo apt upgrade

#### Useful programs:
# Install curl
sudo apt install -y curl
# Install VSCode
sudo apt install software-properties-common apt-transport-https wget -y
wget -q https://packages.microsoft.com/keys/microsoft.asc -O- | sudo apt-key add -
sudo add-apt-repository "deb [arch=amd64] https://packages.microsoft.com/repos/vscode stable main"
sudo apt install code -y
# Install ifconfig
sudo apt install net-tools -y
# Install terminator
sudo apt install terminator -y

#### ROS Humble installation:
# First ensure that the Ubuntu Universe repository is enabled.
sudo apt install software-properties-common -y
sudo add-apt-repository universe -y

# Now add the ROS 2 GPG key with apt.
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Then add the repository to your sources list.
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update again your apt repository caches after setting up the repositories.
sudo apt update
sudo apt upgrade

# Desktop Install (Recommended): ROS, RViz, demos, tutorials.
sudo apt install ros-humble-desktop -y

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
