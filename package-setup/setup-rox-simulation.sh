#!/bin/bash

# exit if any command below fails
set -e
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[0;33m'
WHITE='\033[0;37m'
NC='\033[0m'

echo -e "${GREEN}==========================================================================="
echo -e "Welcome to the support assistant for setting up the ROX Simulation packages"
echo -e "===========================================================================${NC}"

# Check if ROS is sourced
empty_ros_distro=true

if [ "$ROS_DISTRO" == "" ]; then
	echo "ROS Distro is not sourced."
    for dir in /opt/ros/*/; do
        if [ "$(ls -A "$dir")" ]; then
            empty_ros_distro=false
            echo -e "$GREEN"
            echo "  " $(basename "$dir")
            echo -e "$NC"
        fi
    done

    if [ $empty_ros_distro == true ]; then
        echo "ROS 2 is not installed at all"
        echo "Please continue the installation once ROS 2 is installed"
        echo -e "${RED}Abort"
    else
        echo -e "${RED}Please source any one of the ROS Distros above and try again later${NC}"
        echo -e "${RED}Example: source /opt/ros/YOUR_DISTRO/setup.bash${NC}"
        echo -e "${RED}Abort"
        exit 0
    fi
fi

# Get custom workspace base directory
workspace_base=$(bash utils/get_absolute_path.sh)
if [ -z "$workspace_base" ]; then
    echo -e "${RED}Failed to get a valid workspace directory${NC}"
    echo -e "${RED}Abort"
    exit 0
fi
# Create workspace path using this base
directory_root="${workspace_base}/${ROS_DISTRO}_ws"
directory="${directory_root}/src"

if [ -d "${directory_root}" ]; then
    echo "${directory_root} already exists."
    echo -n "Do you want to delete it before continuing? (Y/n)"
    read is_ws_installed

    if [[ "$is_ws_installed" == "y" || "$is_ws_installed" == "Y" ]]; then
        echo -e " Checking permissions..."

        # Check if we have permission to delete
        parent_dir=$(dirname "${directory_root}")
        if [ ! -w "${parent_dir}" ]; then
            echo -e "${YELLOW}You don't have permission to delete this directory.${NC}"
            echo -n "Continue with sudo? (Y/n)"
            read use_sudo

            if [[ "$use_sudo" == "y" || "$use_sudo" == "Y" ]]; then
                echo -e "Deleting ${directory_root}..."
                sudo rm -rf "${directory_root}"
            else
                echo -e "${RED}Abort"
                exit 0
            fi
        else
            echo -e "Deleting ${directory_root}..."
            rm -rf "${directory_root}"
        fi

        echo -e "${GREEN}${directory_root} has been deleted${NC}"
    else
        echo -e "${RED}Abort"
        exit 0
    fi
fi

skip_depend="phidgets_drivers ur_client_library ur_msgs neo_relayboard_v3"

# Install build tool
echo "Installing colcon extensions..."
sudo apt install python3-colcon-common-extensions

# Installing CycloneDDS
echo "Installing CycloneDDS..."
sudo apt install ros-$ROS_DISTRO-rmw-cyclonedds-cpp

#Install xterm

sudo apt install xterm

mkdir -p "$directory"
cd "$directory"

# clone git repos here...
echo "Cloning the necessary repositories..."
git clone --branch $ROS_DISTRO     https://github.com/neobotix/rox.git
git clone --branch $ROS_DISTRO     https://github.com/neobotix/neo_local_planner2.git
git clone --branch $ROS_DISTRO     https://github.com/neobotix/neo_localization2.git
git clone --branch $ROS_DISTRO     https://github.com/neobotix/neo_rox_moveit2.git
git clone --branch master          https://github.com/neobotix/neo_common2
git clone --branch master          https://github.com/neobotix/neo_msgs2
git clone --branch master          https://github.com/neobotix/neo_srvs2
git clone --branch main            https://github.com/neobotix/neo_gz_worlds.git

cd ..

echo -e "${YELLOW}Skipping to install following dependencies:${NC}" $skip_depend
# Install relevant dependencies
rosdep install --from-paths ./src --ignore-src --rosdistro $ROS_DISTRO -r --skip-keys "$skip_depend"

# build workspace
echo "Building the workspace..."
colcon build --symlink-install 

echo -e "The following changes will be made to your ~/.bashrc file:"
echo "  1. Setting LC_NUMERIC=en_US.UTF-8"
echo "  2. Auto-sourcing this workspace on terminal startup"
# Set UTF-8 settings to US
echo "export LC_NUMERIC="en_US.UTF-8" " >> ~/.bashrc

# can add option here
echo "source $directory_root/install/setup.bash" >> ~/.bashrc

echo -e "${YELLOW}Note:${NC} If you have multiple ROS workspaces, auto-sourcing might cause conflicts."

echo -e "${GREEN}Installation successful !!!${NC}"
echo -e "Workspace '${GREEN}${ROS_DISTRO}_ws${NC}' installed at ${GREEN}${workspace_base}${NC}"
exit 0
