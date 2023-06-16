#!/bin/bash
/etc/init.d/ueyeethdrc start
source devel/setup.bash
#source install/setup.bash
rqt &
# roslaunch ueye_cam rgb8.launch
roslaunch camera_tracker tracking_nodelet.launch
