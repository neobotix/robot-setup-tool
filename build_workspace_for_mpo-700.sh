#!/bin/bash

# exit if any command below fails
set -e

# create folders
mkdir -p ~/.config/autostart/

# copy files
cp generic/ROS-Neobotix-Autostart.desktop ~/.config/autostart/
cp generic/startROS.desktop ~/Desktop/
cp mpo-700/ROS_AUTOSTART.sh mpo-700/ros_settings.sh ~/

# add source to .bashrc
echo "source ~/ros_settings.sh" >> ~/.bashrc

rosdep update

source /opt/ros/noetic/setup.bash

cd ~

mkdir -p ros_workspace/src
cd ros_workspace/src

# clone git repos here...
git clone --branch master 		https://github.com/neobotix/neo_mpo_700
git clone --branch master               https://github.com/neobotix/neo_common
git clone --branch master 		https://github.com/neobotix/neo_relayboard_v2
git clone --branch master		https://github.com/neobotix/neo_kinematics_omnidrive.git
git clone --branch master 		https://github.com/neobotix/neo_sick_s300
git clone --branch master 		https://github.com/neobotix/neo_teleop
git clone --branch master 		https://github.com/neobotix/neo_msgs
git clone --branch master 		https://github.com/neobotix/neo_srvs
git clone --branch master 		https://github.com/neobotix/neo_localization
git clone --branch melodic-devel	https://github.com/neobotix/slam_gmapping

# build workspace
cd ..
catkin_make

echo "Installation successful !!!"

exit 0
