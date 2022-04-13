FROM osrf/ros:melodic-desktop
RUN apt-get update && apt-get install -y \    
    ros-melodic-rviz \
    ros-melodic-std-msgs \
    ros-melodic-navigation \
    ros-melodic-industrial-msgs \
	git \
    software-properties-common \
    python-requests			

RUN apt update && apt upgrade -y

# Install OpenCV
RUN echo '=== Installing OpenCV ==='

RUN apt install -y build-essential
RUN apt install -y cmake libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
RUN add-apt-repository --yes "deb http://security.ubuntu.com/ubuntu xenial-security main"
RUN apt update
RUN apt install libjasper1 libjasper-dev
RUN apt install -y  python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev

WORKDIR /
RUN git clone -b 4.2.0 https://github.com/opencv/opencv.git opencv-4.2.0
RUN mkdir -p opencv-4.2.0/build
WORKDIR /opencv-4.2.0/build
RUN cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local ..
RUN make
RUN make install

# Install sources
RUN echo '=== Installing sources ==='
COPY ./src /catkin_ws/src

RUN cd /catkin_ws/src/tracking/data/YOLO/ && wget https://pjreddie.com/media/files/yolov3.weights

RUN cd /catkin_ws/src && \
	git clone -b v0.2.0 --recursive https://github.com/iml130/firos.git

RUN cd /catkin_ws/src && \
	git clone --recursive https://github.com/ramp-eu/Local_Sensing_and_Perception.git

# Build
WORKDIR /catkin_ws

RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; . /catkin_ws/devel/setup.bash; catkin_make'

# Add ROS setup bash to .bashrc
RUN echo 'source /opt/ros/melodic/setup.bash \n source /catkin_ws/devel/setup.bash' >> /root/.bashrc

# Clean up
RUN rm -rf /var/lib/apt/lists/*

COPY ./docker/firos_tracking /catkin_ws/src/firos/config

COPY ./docker/ros_entrypoint.sh /

WORKDIR /

RUN chmod +x /ros_entrypoint.sh

