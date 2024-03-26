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
# Install git
sudo apt install git -y

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
sudo apt install ros-dev-tools -y

# Source ROS 2 setup.bash and set to do it every time bash is opened.
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Create ROS 2 workspace directory
mkdir -p ~/ros_ws/src
cd ~/ros_ws/src

git clone git@github.com:CDEI-Agro/agri_bot.git
git clone -b ros2 --recurse-submodules git@github.com:CDEI-Agro/ouster-ros.git

cd ~/ros_ws

rosdep install --from-paths . --ignore-src -y

sudo apt install ros-humble-turtlebot3*

mkdir -p ~/.gazebo/models

colcon build --symlink-install --packages-select agri_bot

colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
