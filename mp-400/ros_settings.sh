#!/bin/bash

# external network interface name, as given by ifconfig
INTERFACE=wlp2s0

# get ip address of selected external interface
IP_ADDR=$(ip -4 addr show $INTERFACE | grep -oP '(?<=inet\s)\d+(\.\d+){3}')

# if we have an external ip address use it, otherwise use localhost
if [ -z "$IP_ADDR" ]; then
	echo "Failed to detect IP address of interface $INTERFACE, using localhost as fallback!"
	export ROS_IP=127.0.0.1
	export ROS_HOSTNAME=localhost
else
	echo "Detected IP address: $IP_ADDR"
	export ROS_IP=$IP_ADDR
	export ROS_HOSTNAME=$IP_ADDR
fi

source ~/ros_workspace/devel/setup.bash

export ROBOT=mp_400

export MAP_NAME=test_map

