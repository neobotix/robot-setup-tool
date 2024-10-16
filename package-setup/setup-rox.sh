#!/bin/bash

# exit if any command below fails
set -e

# create folders
mkdir -p ~/.config/autostart/

# copy files
cp ../generic/ROS-Neobotix-Autostart.desktop ~/.config/autostart/
cp ../generic/startROS.desktop ~/Desktop/

# Check if ROS is sourced

if [ "$ROS_DISTRO" == "" ];
then
	echo "Installation cannot continue. No ROS sourced, please check if ROS is installed and sourced. Please try again after that!"
	exit 0
fi

# Install build tool
sudo apt install python3-colcon-common-extensions

# Install navigation packages

# Nav2
sudo apt install -y ros-$ROS_DISTRO-navigation2 ros-$ROS_DISTRO-nav2-*

sudo apt install -y ros-$ROS_DISTRO-slam-toolbox

#Teleop-joy
sudo apt-get install -y ros-$ROS_DISTRO-teleop-twist-joy

#Teleop-key
sudo apt-get install -y ros-$ROS_DISTRO-teleop-twist-keyboard

#Topic tools
sudo apt-get install -y ros-$ROS_DISTRO-topic-tools

#Xacro
sudo apt-get install -y ros-$ROS_DISTRO-xacro

#LaserScanner
sudo apt-get install ros-$ROS_DISTRO-sick-safetyscanners2

cd ~

mkdir -p ros2_workspace/src
cd ros2_workspace/src

# clone git repos here...
git clone --branch $ROS_DISTRO     https://github.com/neobotix/rox.git
git clone --branch $ROS_DISTRO     https://github.com/neobotix/neo_nav2_bringup.git
git clone --branch $ROS_DISTRO     https://github.com/neobotix/neo_local_planner2.git
git clone --branch $ROS_DISTRO     https://github.com/neobotix/neo_localization2.git
git clone --branch master          https://github.com/neobotix/neo_common2
git clone --branch master          https://github.com/neobotix/neo_relayboard_v3
git clone --branch main            https://github.com/neobotix/rox_argo_kinematics.git
git clone --branch $ROS_DISTRO     https://github.com/neobotix/neo_teleop2
git clone --branch master          https://github.com/neobotix/neo_msgs2
git clone --branch master          https://github.com/neobotix/neo_srvs2

cd neo_relayboard_v3
#submodule init
git submodule update --init

# install vnx base
sudo dpkg -i vnx-base/x86_64/vnx-base-1.9.6-x86_64-ubuntu-22.04.deb

# build workspace
cd ../..
colcon build --symlink-install 

echo "export LC_NUMERIC="en_US.UTF-8" " >> ~/.bashrc

echo "source ~/ros2_workspace/install/setup.bash" >> ~/.bashrc

echo "Setting up startup scripts"

echo "source ~/ros2_workspace/install/setup.bash" >> ROS_AUTOSTART.sh

echo "sleep 2" >> ROS_AUTOSTART.sh

echo "ros2 launch rox_bringup bringup_launch.py" >> ROS_AUTOSTART.sh

chmod +x ROS_AUTOSTART.sh

mv ROS_AUTOSTART.sh ~/

echo "Installation successful !!!"

exit 0
