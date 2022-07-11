#!/bin/bash

# check for root privileges
uid=$(/usr/bin/id -u) && [ "$uid" = "0" ] || { echo "Run this script as root!"; exit 1; }

sed -i 's/APT::Periodic::Update-Package-Lists "1"/APT::Periodic::Update-Package-Lists "0"/' /etc/apt/apt.conf.d/20auto-upgrades

exit 0
