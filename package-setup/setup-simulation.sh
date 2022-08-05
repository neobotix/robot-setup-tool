#!/bin/bash

# exit if any command below fails
set -e

# Check if ROS is sourced

if [ "$ROS_DISTRO" == "" ];
then
	echo "Installation cannot continue. No ROS sourced, please check if ROS is installed and sourced. Please try again after that!"
	exit 0
fi

# Install build tool
sudo apt install python3-colcon-common-extensions

# Install Gazebo packages

sudo apt-get install ros-$ROS_DISTRO-gazebo-ros ros-$ROS_DISTRO-gazebo-plugins ros-$ROS_DISTRO-gazebo-ros-pkgs

#Install xterm

sudo apt install xterm

# Install navigation packages

# Nav2
sudo apt install -y ros-$ROS_DISTRO-navigation2 ros-$ROS_DISTRO-nav2-*

sudo apt install -y ros-$ROS_DISTRO-slam-toolbox

#Teleop-joy
sudo apt-get install -y ros-$ROS_DISTRO-teleop-twist-joy

#Teleop-key
sudo apt-get install -y ros-$ROS_DISTRO-teleop-twist-keyboard

cd ~

mkdir -p neobotix_workspace/src
cd neobotix_workspace/src

# clone git repos here...
git clone --branch $ROS_DISTRO     https://github.com/neobotix/neo_simulation2.git
git clone --branch $ROS_DISTRO     https://github.com/neobotix/neo_nav2_bringup.git
git clone --branch $ROS_DISTRO     https://github.com/neobotix/neo_local_planner2.git
git clone --branch $ROS_DISTRO     https://github.com/neobotix/neo_localization2.git
git clone --branch master          https://github.com/neobotix/neo_common2
git clone --branch master          https://github.com/neobotix/neo_msgs2
git clone --branch master          https://github.com/neobotix/neo_srvs2

# build workspace
cd ..
colcon build --symlink-install 

echo "export LC_NUMERIC="en_US.UTF-8" " >> ~/.bashrc

echo "source neobotix_workspace/install/setup.bash" >> ~/.bashrc

echo "Installation successful !!!"

echo "Do not forget to set the environment variables"

exit 0