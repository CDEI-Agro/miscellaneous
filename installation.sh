#!/bin/bash

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

# Then add the repository to your sources list.
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update again your apt repository caches after setting up the repositories.
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

# Desktop Install (Recommended): ROS, RViz, demos, tutorials.
sudo apt install ros-humble-desktop -y
sudo apt install ros-dev-tools -y

# Source ROS 2 setup.bash and set to do it every time bash is opened.
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Create ROS 2 workspace directory
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

git clone git@github.com:CDEI-Agro/agri_bot.git

if [[ "$PLATFORM" == "green" || "$PLATFORM" == "red" ]]; then
    # Clone ouster-ros repository
    git clone -b ros2 --recurse-submodules git@github.com:CDEI-Agro/ouster-ros.git

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

cd ~/ros2_ws
echo "source /home/$USER/ros2_ws/install/setup.bash" >> ~/.bashrc


#rosdep install --from-paths . --ignore-src -y

sudo apt install ros-humble-turtlebot3*

mkdir -p ~/.gazebo/models

colcon build --symlink-install --packages-select agri_bot

if [[ "$PLATFORM" == "green" || "$PLATFORM" == "red" ]]; then
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select ouster_sensor_msgs
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select ouster_ros

fi
