#include <ros/ros.h>
#include <camera_tracker/camera_tracker.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "camera_tracker_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    camera_tracker::CameraTracker tfc(nh, pnh);

    ros::spin();

    return 0;
}