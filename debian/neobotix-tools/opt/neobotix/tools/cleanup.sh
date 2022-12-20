#!/bin/sh

username=neobotix

# check for root privileges
uid=$(/usr/bin/id -u) && [ "$uid" = "0" ] || { echo "Run this script as root!"; exit 1; }

# empty trash
rm -rf /home/$username/.local/share/Trash/*

# remove unused packages
apt-get -y autoremove

# delete ros logs
rm -rf /home/$username/.ros/log/*

# delete pilot logs
rm -rf /home/$username/pilot/user/data/logs/*

# delete system logs
rm -rf /var/log/*

# this is needed for journalctl (systemd)
mkdir /var/log/journal

exit 0
