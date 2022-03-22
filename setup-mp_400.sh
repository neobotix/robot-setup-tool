#!/bin/bash

# exit if any command below fails
set -e

# Install navigation packages

# Nav2
sudo apt install -y ros-$ROS_DISTRO-navigation2 ros-$ROS_DISTRO-nav2-*

sudo apt install -y ros-$ROS_DISTRO-slam-toolbox

#Teleop-joy
sudo apt-get install -y ros-$ROS_DISTRO-teleop-twist-joy

#Teleop-key
sudo apt-get install -y ros-$ROS_DISTRO-teleop-twist-keyboard

rosdep update

cd ~

mkdir -p mp_400_workspace/src
cd mp_400_workspace/src

# clone git repos here...
git clone --branch $ROS_DISTRO     https://github.com/neobotix/neo_mp_400-2.git
git clone --branch main            https://github.com/neobotix/neo_nav2_bringup.git
git clone --branch $ROS_DISTRO     https://github.com/neobotix/neo_local_planner2.git
git clone --branch $ROS_DISTRO     https://github.com/neobotix/neo_localization2.git
git clone --branch master          https://github.com/neobotix/neo_common2
git clone --branch $ROS_DISTRO     https://github.com/neobotix/neo_relayboard_v2-2
git clone --branch $ROS_DISTRO     https://github.com/neobotix/neo_kinematics_differential2.git
git clone --branch $ROS_DISTRO     https://github.com/neobotix/neo_sick_s300-2
git clone --branch $ROS_DISTRO     https://github.com/neobotix/neo_teleop2
git clone --branch master          https://github.com/neobotix/neo_msgs2
git clone --branch master          https://github.com/neobotix/neo_srvs2

# build workspace
cd ..
colcon build --symlink-install 

echo "export LC_NUMERIC="en_US.UTF-8" " >> ~/.bashrc

echo "Installation successful !!!"

exit 0