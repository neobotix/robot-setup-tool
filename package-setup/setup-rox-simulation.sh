#!/bin/bash

# exit if any command below fails
set -e
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[0;33m'
WHITE='\033[0;37m'

echo "Welcome to the support assistant for setting up the ROX Simulation packages"
echo "The workspace will be created in the /home/USER/ folder. In the future, we will add the option, to create the workspace in any desired directory"

# Check if ROS is sourced
empty_ros_distro=true
ros_distros=" "
if [ "$ROS_DISTRO" == "" ]; then
	echo "ROS Distro is not sourced."
    for dir in /opt/ros/*/; do
        if [ "$(ls -A "$dir")" ]; then
            empty_ros_distro=false
            echo -e "$GREEN"
            echo "  " $(basename "$dir")
        fi
    done

    if [ $empty_ros_distro == true ]; then
        echo "ROS 2 is not installed at all"
        echo "Please continue the installation once ROS 2 is installed"
        echo "Abort"
    else
        echo -e "${RED} Please source any one of the ROS Distros above and try again later"
        echo -e "${RED} Example: source /opt/ros/YOUR_DISTRO/setup.bash"
        echo "Abort"
        exit 0
    fi
fi

# Check if the client ws is already there in the home folder
cd ~/
is_ws_installed=""

if [ -d "${ROS_DISTRO}_ws" ]; then
    echo "${ROS_DISTRO}_ws already exists."
    echo -n "Do you want to delete it before continuing? (Y/n)"
    read is_ws_installed

    if [[ "$is_ws_installed" == "y" || "$is_ws_installed" == "Y" ]]; then
        echo -e "  Deleting.."
        rm -rf ${ROS_DISTRO}_ws
        echo -e "  ${ROS_DISTRO}_ws has been deleted"
    else
        echo "Abort"
        exit 0
    fi
fi

uni_ans=""
robot_model=""

# Go to home directory
cd ~

skip_depend="phidgets-drivers ur_client_library ur_msgs ur_robot_driver"

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
echo "export IGN_GAZEBO_RESOURCE_PATH=~/"$directory"/rox/:~/"$directory"/neo_gz_worlds/models" >> ~/.bashrc

exit 0