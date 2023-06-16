#!/bin/bash
mkdir -p /root/data/stereo/input/
mkdir -p /root/data/stereo/output/
mkdir -p /root/data/intrinsic/input/images_1/
mkdir -p /root/data/intrinsic/input/images_2/
mkdir -p /root/catkin_ws/devel/share/camera_tracker/data/

chmod +x /root/*.sh

# install ids camera drivers
cd /root/ids_drivers ; tar -xvf ids-software-suite-linux-64-4.96.1-archive.tgz ; ./ueye_4.96.1.2054_amd64.run --auto

# compile ros packages
cd /root/catkin_ws ; catkin_make
echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc
source ~/.bashrc
