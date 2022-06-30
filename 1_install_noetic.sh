#!/bin/bash

# check for root privileges
uid=$(/usr/bin/id -u) && [ "$uid" = "0" ] || { echo "Run this script as root!"; exit 1; }

# check if there are applications running which can interfere with the script
sleep 1
for lock in synaptic update-manager software-center apt-get dpkg aptitude
do
if ps -U root -u root u | grep $lock | grep -v grep > /dev/null;
       then
       echo "Installation won't work. Please close $lock first then try again.";
       exit
fi
done

# exit if any command below fails
set -e

# enable required repos
add-apt-repository restricted
add-apt-repository universe
add-apt-repository multiverse

# update the system sources and software
apt-get clean
apt-get update
apt-get -y upgrade
apt-get -y dist-upgrade
apt-get clean

# install build dependencies
apt-get -y install vim
apt-get -y install git
apt-get -y install cmake
apt-get -y install libgtest-dev
apt-get -y install build-essential

# add packages.ros.org to sources
echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
apt-get update

# install ros packages here...
apt-get -y install ros-noetic-desktop-full python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
apt-get -y install ros-noetic-joy
apt-get -y install ros-noetic-amcl
apt-get -y install ros-noetic-move-base
apt-get -y install ros-noetic-map-server
apt-get -y install ros-noetic-openslam-gmapping
apt-get -y install ros-noetic-tf2-sensor-msgs
apt-get -y install ros-noetic-tf2-geometry-msgs
apt-get -y install ros-noetic-neo-local-planner


rosdep init

echo "Installation successful !!!"

exit 0
