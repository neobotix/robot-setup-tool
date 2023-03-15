This package contains scripts that are useful for the initial setup of the robots, due to first time commissioning or [upgrade of the OS](https://neobotix-docs.de/ros/migration/index.html). Usually it is not necessary to run these scripts again at a later point of time. However, there may be cases where it could be useful. For such cases following scripts can be found under `/opt/neobotix/tools/`.

**cleanup.sh**
---
*Cleanup the OS before backup.*

| WARNING: Use this script at yout own risk. It will remove unused packages, system logs and bash history. |
| --- |

**disable_auto_upgrades.sh**
---
*Disable unattended updates. Run this script if disabling automatic updates in system settings is not working as expected.*

**display_hotplug.sh**
---
*This script is handy when internal (usually not connected) output is detected and configured.
When needed triggered automatically on specified events.*

**enable_can0.sh**
---
*Configures the CAN interface.
When needed triggered automatically on specified events.*

**enable_multicast_lo.sh**
---
*Configures multicast for the loopback interface.*

**sync_time.sh**
---
*Grabs the date from google.com and sets it as current date on the PC.*

**xfce_tweaks.sh**
---
*Configures XFCE desktop environment for the use on the robot.*
