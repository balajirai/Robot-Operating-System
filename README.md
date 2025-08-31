# ROS2 Installation
```bash
# Set up locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" # If using Ubuntu derivates use $UBUNTU_CODENAME
sudo dpkg -i /tmp/ros2-apt-source.deb

# Install ROS 2
sudo apt update && sudo apt upgrade
# sudo apt install ros-humble-desktop         # Provide GUI tools (But takes around 2GB+)

# If ROS Humble Desktop is not prefered then,
# ROS-Base Install (Bare Bones)
# Development tools, No GUI tools
sudo apt install ros-humble-ros-base
sudo apt install ros-dev-tools

# Source ROS 2 setup file (or add it to ~/.bashrc)
source /opt/ros/humble/setup.bash

# Install colcon (build tool) and other tools
sudo apt install python3-colcon-common-extensions python3-rosdep2
```

For more info, please refer to ros2 [official documentation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)