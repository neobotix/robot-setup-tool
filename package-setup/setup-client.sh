#!/bin/bash

# exit if any command below fails
set -e
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[0;33m'
WHITE='\033[0;37m'

# Check if ROS is sourced

if [ "$ROS_DISTRO" == "" ];
then
	echo "Installation cannot continue. No ROS sourced, please check if ROS is installed and sourced. Please try again after that!"
	exit 0
fi

echo "Welcome to the support assistant for setting up the client PC Neobotix packages"

uni_ans=""
robot_model=""

# Go to home directory
cd ~

mkdir -p client_workspace/src
cd client_workspace/src

while [[ "$robot_model" != "mp_400" && 
        "$robot_model" != "mp_500" && 
        "$robot_model" != "mpo_500" && 
        "$robot_model" != "mpo_700" &&
        "$robot_model" != "rox" ]]; do
	echo -e "${GREEN} Choose your robot (mp_400/mp_500/mpo_500/mpo_700/rox)"

    read robot_model

    echo -e "${WHITE}"

    if [ "$robot_model" == "rox" ]; then
		git clone --branch $ROS_DISTRO     https://github.com/neobotix/rox.git || { true; echo -e ${YELLOW} "Continuing with the installation"; }
	elif [[ "$robot_model" == "mp_400" || "$robot_model" == "mp_500" || "$robot_model" == "mpo_500" || "$robot_model" == "mpo_700" ]]; then
        git clone --branch $ROS_DISTRO     https://github.com/neobotix/neo_$robot_model-2.git || { true; echo -e ${YELLOW} "Continuing with the installation"; }
    else
        echo -e "${RED} Please select a valid option"
    fi
done

sudo rosdep init || { true; echo -e "${YELLOW} rosdep init is not required"; }
rosdep update

# Install build tool
echo "Installing colcon extensions"
sudo apt install python3-colcon-common-extensions

# Installing CycloneDDS
echo "Installing CycloneDDS"
sudo apt install ros-$ROS_DISTRO-cyclonedds

#Install xterm - useful when 
sudo apt install xterm

git clone --branch master          https://github.com/neobotix/neo_msgs2 || { true; echo -e "${YELLOW} Continuing with the installation"; }
git clone --branch master          https://github.com/neobotix/neo_srvs2 || { true; echo -e "${YELLOW} Continuing with the installation"; }

if [ "$ROS_DISTRO" == "iron" ]; then
	git clone --branch $ROS_DISTRO     https://github.com/neobotix/neo_nav2_bringup.git || { true; echo -e "${YELLOW} Continuing with the installation"; }
fi

cd ~/client_workspace

# Install relevant dependencies
rosdep install --from-paths ./src/rox/rox_navigation/ ./src/rox/rox_description --ignore-src --rosdistro $ROS_DISTRO -r -y || { true; echo -e "${YELLOW} Finishing the setup"; }

cd ~/client_workspace/src

if [[ "$robot_model" == "mpo_500" || "$robot_model" == "mpo_700" || "$robot_model" == "rox" ]]; then
	while [[ "$uni_ans" != "y" && "$uni_ans" != "n" ]]; do
		echo -e "${GREEN} Universal robots ? (y/n)"
		read uni_ans

        if [ "$uni_ans" == "y" ]; then
            if [[ "$robot_model" == "mpo_500" || "$robot_model" == "mpo_700" ]]; then
                git clone --branch main https://github.com/neobotix/neo_mpo_moveit2.git || { true; echo -e ${YELLOW} "Continuing with the installation"; }
                rosdep install --from-paths ./neo_mpo_moveit2 --ignore-src --rosdistro $ROS_DISTRO -r -y || { true; echo -e "${YELLOW} Finishing the setup"; }
            elif [ "$robot_model" == "rox" ]; then
                git clone --branch main https://github.com/neobotix/neo_rox_moveit2.git || { true; echo -e ${YELLOW} "Continuing with the installation"; }
                rosdep install --from-paths ./neo_rox_moveit2 --ignore-src --rosdistro $ROS_DISTRO -r -y || { true; echo -e "${YELLOW} Finishing the setup"; }
            else
                uni_ans = "n"
            fi
        fi
    done
fi

cd ~/client_workspace

echo -e "${WHITE}"

# build workspace
colcon build --symlink-install 

echo "source ~/client_workspace/install/setup.bash" >> ~/.bashrc

echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc

echo "Installation successful !!!"

exit 0
