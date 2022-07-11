#!/bin/bash

# autostart command:
# sh -c "sleep 10 ; bash /home/neobotix/.neobotix/disply_hotplug.sh"

# udev rule for display hotplug:
# echo 'ACTION=="change", SUBSYSTEM=="drm", ENV{HOTPLUG}=="1", RUN+="/home/neobotix/.neobotix/display_hotplug.sh"' >> /etc/udev/rules.d/99-display-hotplug.rules

vga="$(cat /sys/class/drm/card0-VGA-1/status)"
dvi="$(cat /sys/class/drm/card0-DVI-D-1/status)"
hdmi1="$(cat /sys/class/drm/card0-HDMI-A-1/status)"
hdmi2="$(cat /sys/class/drm/card0-HDMI-A-2/status)"
edp1="$(cat /sys/class/drm/card0-eDP-1/status)"

# This script is called as root and without X knowledge
export DISPLAY=:0.0
export XAUTHORITY=/home/neobotix/.Xauthority

xrandr --auto

if [ "${vga}" = connected ]; then

	xrandr --output VGA-1 --auto
	xrandr --output eDP-1 --off

elif [ "${dvi}" = connected ]; then

	xrandr --output DVI-D-1 --auto
	xrandr --output eDP-1 --off

elif [ "${hdmi1}" = connected ]; then

	xrandr --output HDMI-1 --auto
	xrandr --output eDP-1 --off

elif [ "${hdmi2}" = connected ]; then

	xrandr --output HDMI-2 --auto
	xrandr --output eDP-1 --off

else
        xrandr --fb 1024x768
fi
