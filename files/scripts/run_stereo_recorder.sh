#!/bin/bash
/etc/init.d/ueyeethdrc start
source devel/setup.bash
#source install/setup.bash

rm -rf /root/data/intrinsic/input/images_1/*.png
rm -rf /root/data/intrinsic/input/images_2/*.png
rm -rf /root/data/stereo/input/*.avi

rqt &
# roslaunch ueye_cam rgb8.launch
roslaunch stereo_camera_calibration stereo_camera_recorder_nodelet.launch
