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
realsense_ans=""
skip_depend=""
arm_type=" "
use_imu="False"
use_d435="False"

while [[ "$robot_model" != "mp_400" && "$robot_model" != "mp_500" && "$robot_model" != "mpo_500" && "$robot_model" != "mpo_700" ]]; do
	echo "Choose your robot (mp_400/mp_500/mpo_500/mpo_700)"
	read robot_model
	if [[ "$robot_model" == "mp_400" || "$robot_model" == "mp_500" ]]; then
		echo "neo_kinematics_differential2 package will be cloned"
	elif [ "$robot_model" == "mpo_500" ]; then
		echo "neo_kinematics_mecanum2 package will be cloned"
	elif [ "$robot_model" == "mpo_700" ]; then
		echo "neo_kinematics_omnidrive2 package will be cloned"
	else
		echo "Wrong option - Please try again"
	fi
done

if [[ "$robot_model" == "mpo_500" || "$robot_model" == "mpo_700" ]]; then
	while [[ "$uni_ans" != "y" && "$uni_ans" != "n" ]]; do
		echo "Universal robots ? (y/n)"
		read uni_ans
		if [ "$uni_ans" == "n" ]; then
			skip_depend+="ur_client_library ur_msgs ur_description ur_robot_driver "
		elif [ "$uni_ans" == "y" ]; then
			echo "Universal robots dependencies will be installed and added to autstart"
			while [[ "$arm_type" != "ur10" && "$arm_type" != "ur10e" && "$arm_type" != "ur5" && "$arm_type" != "ur5e" ]]; do
				echo "arm_type? (ur10/ur10e/ur5/ur5e)"
				read arm_type
			done
		else
			echo "Wrong option - Please try again"
		fi
	done
fi

while [[ "$phi_ans" != "y" && "$phi_ans" != "n" ]]; do
	echo "Phidget IMU ? (y/n)"
	read phi_ans
	if [ "$phi_ans" == "n" ]; then
		skip_depend+="phidgets-drivers"
		use_imu="False"
	elif [ "$phi_ans" == "y" ]; then
		echo "Phidget IMU dependencies will be installed and added to autstart"
		use_imu="True"
	else
		echo "Wrong option - Please try again"
	fi
done

while [[ "$realsense_ans" != "y" && "$realsense_ans" != "n" ]]; do
	echo "Realsense Camera (URDF only supports D435i) ? (y/n)"
	read realsense_ans
	if [ "$realsense_ans" == "n" ]; then
		skip_depend+="realsense2_camera realsense2_camera_msgs realsense2_description"
		use_d435="False"
	elif [ "$realsense_ans" == "y" ]; then
		echo "Realsense camera dependencies will be installed and added to autstart"
		use_d435="True"
	else
		echo "Wrong option - Please try again"
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

cd ~

mkdir -p ros2_workspace/src
cd ros2_workspace/src

# clone git repos here...
git clone --branch $ROS_DISTRO     https://github.com/neobotix/neo_$robot_model-2.git
git clone --branch $ROS_DISTRO     https://github.com/neobotix/neo_local_planner2.git
git clone --branch $ROS_DISTRO     https://github.com/neobotix/neo_localization2.git
git clone --branch master          https://github.com/neobotix/neo_common2
git clone --branch $ROS_DISTRO     https://github.com/neobotix/neo_relayboard_v2-2
git clone --branch $ROS_DISTRO     https://github.com/neobotix/neo_sick_s300-2
git clone --branch $ROS_DISTRO     https://github.com/neobotix/neo_teleop2
git clone --branch master          https://github.com/neobotix/neo_msgs2
git clone --branch master          https://github.com/neobotix/neo_srvs2
git clone https://github.com/neobotix/joystick_drivers.git

if [ "$ROS_DISTRO" == "iron" ]; then
	git clone --branch $ROS_DISTRO     https://github.com/neobotix/neo_nav2_bringup.git
fi

if [[ "$robot_model" == "mp_400" || "$robot_model" == "mp_500" ]]; then
	git clone --branch main https://github.com/neobotix/neo_kinematics_differential2.git
elif [ "$robot_model" == "mpo_500" ]; then
	git clone --branch $ROS_DISTRO https://github.com/neobotix/neo_kinematics_mecanum2.git
elif [ "$robot_model" == "mpo_700" ]; then
	git clone --branch $ROS_DISTRO https://github.com/neobotix/neo_kinematics_omnidrive2.git
fi

if [ "$uni_ans" == "y" ]; then
	git clone --branch main https://github.com/neobotix/neo_mpo_moveit2.git
fi

# build workspace
cd ..

echo "Installing dependencies, skipping the following" $skip_depend
# Install relevant dependencies
rosdep install --from-paths ./src --ignore-src --rosdistro $ROS_DISTRO -r -y --skip-keys "$skip_depend"

colcon build --symlink-install 

echo "export LC_NUMERIC="en_US.UTF-8" " >> ~/.bashrc

echo "source ~/ros2_workspace/install/setup.bash" >> ~/.bashrc

echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc

echo "Setting up startup scripts"

echo "source ~/ros2_workspace/install/setup.bash" >> ROS_AUTOSTART.sh

echo "sleep 2" >> ROS_AUTOSTART.sh

echo "ros2 launch neo_"$robot_model"-2 bringup.launch.py arm_type:="$arm_type" use_imu:="$use_imu" use_d435:="$use_d435>> ROS_AUTOSTART.sh

chmod +x ROS_AUTOSTART.sh

mv ROS_AUTOSTART.sh ~/

echo "Installation successful !!!"

exit 0
