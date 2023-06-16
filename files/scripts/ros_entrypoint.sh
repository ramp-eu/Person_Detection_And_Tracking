#!/bin/bash
set -e

echo SP Local : OPIL version 3 is now running

# firos setup
sed -e "s/LOCALHOST/$LOCALHOST/g" -e "s/FIWAREHOST/$FIWAREHOST/g" /root/catkin_ws/src/firos/config/config.json.template > /root/catkin_ws/src/firos/config/config.json

# cameras daemon
#/etc/init.d/ueyeethdrc start

# setup ros environment
source "/root/.bashrc"
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/catkin_ws/devel/setup.bash"
cd /root/catkin_ws/ && rosrun firos core.py --conf src/firos/config/
#exec roslaunch camera_tracker tracking_nodelet.launch
#exec  "$@"
