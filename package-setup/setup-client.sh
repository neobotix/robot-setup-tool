#!/bin/bash

# exit if any command below fails
set -e
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[0;33m'
WHITE='\033[0;37m'

echo "Welcome to the support assistant for setting up the client PC Neobotix packages"
echo "The workspace will be created in the /home/USER/ folder. In the future, we will add the option, to create the workspace in any desired directory"

# Check if the client ws is already there in the home folder
cd ~/
is_ws_installed=""

if [ -d "client_nbx_ws" ]; then
    echo "client_nbx_ws already exists."
    echo -n "Do you want to delete it before continuing? (Y/n)"
    read is_ws_installed

    if [[ "$is_ws_installed" == "y" || "$is_ws_installed" == "Y" ]]; then
        echo -e "  Deleting.."
        rm -rf "client_nbx_ws"
        echo -e "  client_nbx_ws has been deleted"
    else
        echo "Abort"
        exit 0
    fi
fi

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

uni_ans=""
robot_model=""

# Go to home directory
cd ~

echo "Creating a client_nbx workspace"

mkdir -p client_nbx_ws/src
cd client_nbx_ws/src

while [[ "$robot_model" != "mp_400" && 
        "$robot_model" != "mp_500" && 
        "$robot_model" != "mpo_500" && 
        "$robot_model" != "mpo_700" &&
        "$robot_model" != "rox" ]]; do
	echo -e -n "${GREEN}  Choose your robot (mp_400/mp_500/mpo_500/mpo_700/rox):"

	read robot_model

	echo -e "${WHITE}"

	if [ "$robot_model" == "rox" ]; then
		git clone --branch $ROS_DISTRO     https://github.com/neobotix/rox.git
	elif [[ "$robot_model" == "mp_400" || "$robot_model" == "mp_500" || "$robot_model" == "mpo_500" || "$robot_model" == "mpo_700" ]]; then
		git clone --branch $ROS_DISTRO     https://github.com/neobotix/neo_$robot_model-2.git
	else
		echo -e "${RED} Please select a valid option"
	fi
done

echo "Performing rosdep initialization and update"
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

git clone --branch master          https://github.com/neobotix/neo_msgs2
git clone --branch master          https://github.com/neobotix/neo_srvs2

cd ~/client_nbx_ws

# Install relevant dependencies
rosdep install --from-paths ./src/rox/rox_navigation/ ./src/rox/rox_description --ignore-src --rosdistro $ROS_DISTRO -r -y || { true; echo -e "${YELLOW} Finishing the setup"; }

cd ~/client_nbx_ws/src

if [[ "$robot_model" == "mpo_500" || "$robot_model" == "mpo_700" || "$robot_model" == "rox" ]]; then
	while [[ "$uni_ans" != "y" && "$uni_ans" != "n" ]]; do
		echo -e "${GREEN} Universal robots ? (y/n)"
		read uni_ans

        if [ "$uni_ans" == "y" ]; then
            if [[ "$robot_model" == "mpo_500" || "$robot_model" == "mpo_700" ]]; then
                git clone --branch main https://github.com/neobotix/neo_mpo_moveit2.git
                rosdep install --from-paths ./neo_mpo_moveit2 --ignore-src --rosdistro $ROS_DISTRO -r -y || { true; echo -e "${YELLOW} Finishing the setup"; }
            elif [ "$robot_model" == "rox" ]; then
                git clone --branch main https://github.com/neobotix/neo_rox_moveit2.git
                rosdep install --from-paths ./neo_rox_moveit2 --ignore-src --rosdistro $ROS_DISTRO -r -y || { true; echo -e "${YELLOW} Finishing the setup"; }
            else
                uni_ans = "n"
            fi
        fi
    done
fi

cd ~/client_nbx_ws

echo -e "${WHITE}"

# build workspace
colcon build --symlink-install 

echo "source ~/client_nbx_ws/install/setup.bash" >> ~/.bashrc

echo "Setting CycloneDDS as the Middleware"

echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc

echo "Installation successful !!!"

exit 0
