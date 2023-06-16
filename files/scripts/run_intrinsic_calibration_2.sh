#!/bin/bash
#/etc/init.d/ueyeethdrc start
source devel/setup.bash
#source install/setup.bash

roslaunch stereo_camera_calibration intrinsic_calibration_cam2.launch
