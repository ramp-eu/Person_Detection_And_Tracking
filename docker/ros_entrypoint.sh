#!/bin/bash
set -e

echo SP Local : OPIL version 3 is now running
# firos setup
sed -e "s/LOCALHOST/$HOST/g" -e "s/FIWAREHOST/$FIWAREHOST/g" /catkin_ws/src/firos/config/config.json.template > /catkin_ws/src/firos/config/config.json


# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/catkin_ws/devel/setup.sh"


exec roslaunch tracking tracking.launch


exec  "$@"
