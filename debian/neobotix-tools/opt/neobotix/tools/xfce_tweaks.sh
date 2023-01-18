#!/bin/bash

if [ $SUDO_USER ]; then user=$SUDO_USER; else user=`whoami`; fi

# Disable compositor
xfconf-query -c xfwm4 -p /general/use_compositing -t bool -s false

# Add keyboard language switcher to panel
xfce4-panel --add=xkb

# Disable lockscreen
xset s 0 0
xset s off -dpms 
xfconf-query -c xfce4-session -p /shutdown/LockScreen -s false
xfconf-query -c xfce4-power-manager -p /xfce4-power-manager/lock-screen-suspend-hibernate -s false

# Disable power saving
gsettings set org.gnome.settings-daemon.plugins.power ambient-enabled false
gsettings set org.gnome.settings-daemon.plugins.power idle-dim false
gsettings set org.gnome.settings-daemon.plugins.power sleep-inactive-ac-type 'nothing'
gsettings set org.gnome.settings-daemon.plugins.power sleep-inactive-battery-type 'nothing'

# Disable saved sessions
rm -rf /home/$user/.cache/sessions/* && chmod 500 /home/$user/.cache/sessions

if [ -f /home/$user/.config/autostart/xfce_tweaks.desktop ]
then
  rm /home/$user/.config/autostart/xfce_tweaks.desktop
fi

exit 0
