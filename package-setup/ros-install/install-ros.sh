#!/bin/bash

# Exit if any command below fails
set -e

echo "=== 1. Setting up locale ==="
locale  # check for UTF-8

sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings

echo "=== 2. Enabling required repositories ==="
sudo apt install -y software-properties-common
sudo add-apt-repository -y universe

sudo apt update && sudo apt install -y curl
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

echo "=== 3. Installing development tools (optional) ==="
sudo apt update && sudo apt install -y ros-dev-tools

echo "=== 4. Installing ROS 2 Lyrical Desktop ==="
sudo apt update
sudo apt upgrade -y
sudo apt install -y ros-lyrical-desktop

echo "=== 5. Setting up environment ==="
source /opt/ros/lyrical/setup.bash
if ! grep -Fxq "source /opt/ros/lyrical/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/lyrical/setup.bash" >> ~/.bashrc
fi

echo "ROS 2 Lyrical Desktop installation complete!"
