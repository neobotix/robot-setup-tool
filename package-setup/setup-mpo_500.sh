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

# Nav2
sudo apt install -y ros-$ROS_DISTRO-navigation2 ros-$ROS_DISTRO-nav2-*

sudo apt install -y ros-$ROS_DISTRO-slam-toolbox

#Teleop-joy
sudo apt-get install -y ros-$ROS_DISTRO-teleop-twist-joy

#Teleop-key
sudo apt-get install -y ros-$ROS_DISTRO-teleop-twist-keyboard

#Topic tools
sudo apt-get install -y ros-$ROS_DISTRO-topic-tools

cd ~

mkdir -p mpo_500_workspace/src
cd mpo_500_workspace/src

# clone git repos here...
git clone --branch $ROS_DISTRO     https://github.com/neobotix/neo_mpo_500-2.git
git clone --branch $ROS_DISTRO     https://github.com/neobotix/neo_nav2_bringup.git
git clone --branch $ROS_DISTRO     https://github.com/neobotix/neo_local_planner2.git
git clone --branch $ROS_DISTRO     https://github.com/neobotix/neo_localization2.git
git clone --branch master          https://github.com/neobotix/neo_common2
git clone --branch $ROS_DISTRO     https://github.com/neobotix/neo_relayboard_v2-2
git clone --branch $ROS_DISTRO     https://github.com/neobotix/neo_kinematics_mecanum2.git
git clone --branch $ROS_DISTRO     https://github.com/neobotix/neo_sick_s300-2
git clone --branch $ROS_DISTRO     https://github.com/neobotix/neo_teleop2
git clone --branch master          https://github.com/neobotix/neo_msgs2
git clone --branch master          https://github.com/neobotix/neo_srvs2

# build workspace
cd ..
colcon build --symlink-install 

echo "export LC_NUMERIC="en_US.UTF-8" " >> ~/.bashrc

echo "source ~/mpo_500_workspace/install/setup.bash" >> ~/.bashrc

echo "Setting up startup scripts"

echo "source ~/mpo_500_workspace/install/setup.bash" >> ROS_AUTOSTART.sh

echo "sleep 2" >> ROS_AUTOSTART.sh

echo "ros2 launch neo_mpo_500-2 bringup.launch.py" >> ROS_AUTOSTART.sh

chmod +x ROS_AUTOSTART.sh

mv ROS_AUTOSTART.sh ~/

echo "Installation successful !!!"

exit 0