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

echo "Welcome to the setup of your MP robot, please select the dependencies that are required for your robot"

uni_ans=""
phi_ans=""
skip_depend=""

while [[ "$uni_ans" != "y" && "$uni_ans" != "n" ]]; do
	echo "Universal robots ? (y/n)"

	read uni_ans

	if [ "$uni_ans" == "n" ]; then
		skip_depend+="ur_client_library ur_msgs ur_description ur_robot_driver "
	elif [ "$uni_ans" == "y" ]; then
		echo "Universal robots dependencies will be installed"
	else
		echo "Wrong option - Please try again"
	fi

done

while [[ "$phi_ans" != "y" && "$phi_ans" != "n" ]]; do
	echo "Phidget IMU ? (y/n)"

	read phi_ans

	if [ "$phi_ans" == "n" ]; then
		skip_depend+="phidgets-drivers"
	elif [ "$phi_ans" == "y" ]; then
		echo "Phidget IMU dependencies will be installed"
	else
		echo "Wrong option - Please try again"
	fi

done

while [[ "$robot_model" != "mp_400" && "$robot_model" != "mp_500" && "$robot_model" != "mpo_500" && "$robot_model" != "mpo_700" ]]; do

	echo "Choose your robot (mp_400/mp_500/mpo_500/mpo_700)"

	read robot_model

	if [ "$robot_model" == "mp_400" || "$robot_model" == "mp_500" ]; then
		echo "neo_kinematics_differential2 package will be cloned"
	elif [ "$robot_model" == "mpo_500" ]; then
		echo "neo_kinematics_mecanum2 package will be cloned"
	elif [ "$robot_model" == "mpo_700" ]; then
		echo "neo_kinematics_omnidrive2 package will be cloned"
	else
		echo "Wrong option - Please try again"
	fi

done

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

cd ~

mkdir -p ros2_workspace/src
cd ros2_workspace/src

# clone git repos here...
git clone --branch $ROS_DISTRO     https://github.com/neobotix/neo_mpo_700-2.git
git clone --branch $ROS_DISTRO     https://github.com/neobotix/neo_nav2_bringup.git
git clone --branch $ROS_DISTRO     https://github.com/neobotix/neo_local_planner2.git
git clone --branch $ROS_DISTRO     https://github.com/neobotix/neo_localization2.git
git clone --branch master          https://github.com/neobotix/neo_common2
git clone --branch $ROS_DISTRO     https://github.com/neobotix/neo_relayboard_v2-2
git clone --branch $ROS_DISTRO     https://github.com/neobotix/neo_sick_s300-2
git clone --branch $ROS_DISTRO     https://github.com/neobotix/neo_teleop2
git clone --branch master          https://github.com/neobotix/neo_msgs2
git clone --branch master          https://github.com/neobotix/neo_srvs2

if [ "$robot_model" == "mp_400" || "$robot_model" == "mp_500" ]; then
	git clone --branch main https://github.com/neobotix/neo_kinematics_differential2.git
elif [ "$robot_model" == "mpo_500" ]; then
	git clone --branch $ROS_DISTRO https://github.com/neobotix/neo_kinematics_mecanum2.git
elif [ "$robot_model" == "mpo_700" ]; then
	git clone --branch $ROS_DISTRO https://github.com/neobotix/neo_kinematics_omnidrive2.git
fi

# build workspace
cd ..
colcon build --symlink-install 

echo "export LC_NUMERIC="en_US.UTF-8" " >> ~/.bashrc

echo "source ~/ros2_workspace/install/setup.bash" >> ~/.bashrc

echo "Setting up startup scripts"

echo "source ~/ros2_workspace/install/setup.bash" >> ROS_AUTOSTART.sh

echo "sleep 2" >> ROS_AUTOSTART.sh

if [ "$robot_model" == "mpo_700" ]; then
	echo "ros2 launch neo_mpo_700-2 bringup.launch.py" >> ROS_AUTOSTART.sh
fi

chmod +x ROS_AUTOSTART.sh

mv ROS_AUTOSTART.sh ~/

echo "Installation successful !!!"

exit 0