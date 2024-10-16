#!/bin/bash

# exit if any command below fails
set -e

# Check if ROS is sourced

if [ "$ROS_DISTRO" == "" ];
then
	echo "Installation cannot continue. No ROS sourced, please check if ROS is installed and sourced. Please try again after that!"
	exit 0
fi

if [ "$ROS_DISTRO" == "Humble" ];
then
	echo "Installation cannot continue. ROX Simulation is only supported in Iron and Jazzy"
	exit 0
fi

echo "Welcome to the setup of the ROX simulation ws"

skip_depend="phidgets-drivers ur_client_library ur_msgs ur_description ur_robot_driver"

# Install build tool
sudo apt install python3-colcon-common-extensions

#Install xterm

sudo apt install xterm

cd ~

directory="${ROS_DISTRO}_ws/src"
directory_root="${ROS_DISTRO}_ws"

mkdir -p "$directory"
cd "$directory"

# clone git repos here...
git clone --branch $ROS_DISTRO     https://github.com/neobotix/rox.git
git clone --branch $ROS_DISTRO     https://github.com/neobotix/neo_nav2_bringup.git
git clone --branch $ROS_DISTRO     https://github.com/neobotix/neo_local_planner2.git
git clone --branch $ROS_DISTRO     https://github.com/neobotix/neo_localization2.git
git clone --branch master          https://github.com/neobotix/neo_common2
git clone --branch master          https://github.com/neobotix/neo_msgs2
git clone --branch master          https://github.com/neobotix/neo_srvs2
git clone --branch main            https://github.com/neobotix/neo_gz_worlds.git

cd ..

echo "skipping to install following dependencies:" $skip_depend
# Install relevant dependencies
rosdep install --from-paths ./src --ignore-src --rosdistro $ROS_DISTRO -r --skip-keys "$skip_depend"

# build workspace
colcon build --symlink-install 

echo "export LC_NUMERIC="en_US.UTF-8" " >> ~/.bashrc

echo "source ~/"$directory_root"/install/setup.bash" >> ~/.bashrc

echo "Installation successful !!!"

# Temporarily setting up the GZ_RESOURCE_PATH from here
echo "export GZ_SIM_RESOURCE_PATH=~/"$directory"/rox/:~/"$directory"/neo_gz_worlds/models" >> ~/.bashrc

exit 0
