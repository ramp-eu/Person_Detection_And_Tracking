#include <ros/ros.h>
#include <stereo_camera_calibration/stereo_camera_recorder.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "stereo_camera_recorder_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    stereo_camera_calibration::StereoCameraRecorder scr(nh, pnh);

    ros::spin();

    return 0;
}