#!/bin/bash

# Parse command-line arguments
while [[ "$#" -gt 0 ]]; do
    case $1 in
        -p|--platform) PLATFORM="$2"; shift ;;
        *) echo "Unknown parameter passed: $1"; exit 1 ;;
    esac
    shift
done

# Set default value if argument is not provided or invalid
PLATFORM="${PLATFORM:-default}"

# Add the universe repository
sudo add-apt-repository universe

# Install necessary tools and update apt repositories
sudo apt update
sudo apt install -y curl software-properties-common apt-transport-https wget net-tools terminator

# Install the ROS keyring
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add the ROS 2 repository to your sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update again after adding the new repository
sudo apt update
sudo apt upgrade -y


#### Useful programs:
# Install Visual Studio Code
wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor | sudo tee /usr/share/keyrings/vscode.gpg > /dev/null
sudo add-apt-repository "deb [arch=amd64 signed-by=/usr/share/keyrings/vscode.gpg] https://packages.microsoft.com/repos/vscode stable main"
sudo apt install -y code

# Install ifconfig
sudo apt install net-tools -y
# Install terminator
sudo apt install terminator -y

# Desktop Install (Recommended): ROS, RViz, demos, tutorials.
sudo apt install -y ros-humble-desktop ros-dev-tools

# Ensure rosdep is installed and initialized
sudo apt install -y python3-rosdep
sudo rosdep init
rosdep update

# Source ROS 2 setup.bash and set to do it every time bash is opened.
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> $HOME/.bashrc

# Create ROS 2 workspace directory
mkdir -p $HOME/ros2_ws/src
cd $HOME/ros2_ws/src

git clone git@gitlab.upc.edu:robot_navigation/agro-moby.git
git clone git@gitlab.upc.edu:robot_navigation/ublox_dual.git -b ros2

if [[ "$PLATFORM" == "green" || "$PLATFORM" == "red" ]]; then
    # Clone ouster-ros repository
    git clone -b ros2 --recurse-submodules git@gitlab.upc.edu:robot_navigation/ouster-ros.git
    # clone sick lidar repositories
    git clone git@gitlab.upc.edu:robot_navigation/libsick_ldmrs.git
    git clone -b master git@gitlab.upc.edu:robot_navigation/sick_scan_xd.git

    # Add export ROS_DOMAIN_ID=1 to ~/.bashrc if PLATFORM is 'green'
    if [[ "$PLATFORM" == "green" ]]; then
        echo 'export ROS_DOMAIN_ID=1' >> ~/.bashrc
    fi
        # Add export ROS_DOMAIN_ID=2 to ~/.bashrc if PLATFORM is 'red'
    if [[ "$PLATFORM" == "red" ]]; then
        echo 'export ROS_DOMAIN_ID=2' >> ~/.bashrc
    fi
fi

if [[ "$PLATFORM" == "default" ]]; then
  cat <<EOT >> ~/.bashrc

  # Function to set MOBY model based on color
  set_moby_model() {
      case "\$1" in
          "GREEN")
              export ROS_DOMAIN_ID=1
              echo "MOBY model set to GREEN"
              ;;
          "RED")
              export ROS_DOMAIN_ID=2
              echo "MOBY model set to RED"
              ;;
          *)
              echo "Invalid robot color. Please specify either GREEN or RED."
              ;;
      esac
  }
EOT
fi

cd $HOME/ros2_ws
echo "source $HOME/ros2_ws/install/setup.bash" >> $HOME/.bashrc


rosdep install --from-paths . --ignore-src -y

sudo apt install ros-humble-turtlebot3*

mkdir -p ~/.gazebo/models

colcon build --symlink-install --packages-select agri_bot
colcon build --symlink-install --packages-select ublox_msgs ublox_serialization ublox_gps ublox

if [[ "$PLATFORM" == "green" || "$PLATFORM" == "red" ]]; then
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select ouster_sensor_msgs
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select ouster_ros
    colcon build --symlink-install --packages-select libsick_ldmrs --event-handlers console_direct+
    colcon build --symlink-install --packages-select sick_scan_xd --cmake-args " -DROS_VERSION=2" " -DLDMRS=0" --event-handlers console_direct+
fi
