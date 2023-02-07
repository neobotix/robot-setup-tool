#!/bin/sh

set -e

if [ $SUDO_USER ]; then user=$SUDO_USER; else user=`whoami`; fi

# check for root privileges
uid=$(/usr/bin/id -u) && [ "$uid" = "0" ] || { echo "Run this script as root!"; exit 1; }

# empty trash
rm -rf /home/$user/.local/share/Trash/*

# remove unused packages
apt-get -y autoremove

# delete ros logs
rm -rf /home/$user/.ros/log/*

# delete pilot logs
rm -rf /home/$user/pilot/user/data/logs/*

# delete system logs
rm -rf /var/log/*

# this is needed for journalctl (systemd)
mkdir /var/log/journal

# clear bash history
>/home/$user/.bash_history
sudo -u$user bash -c "unset HISTFILE; history -c && history -w;"

exit 0
