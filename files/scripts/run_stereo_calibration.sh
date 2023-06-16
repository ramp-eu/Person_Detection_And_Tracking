#!/bin/bash
/etc/init.d/ueyeethdrc start
source devel/setup.bash
#source install/setup.bash

rm -rf /root/data/stereo/output/*.png

roslaunch stereo_camera_calibration stereo_calibration.launch
