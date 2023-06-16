#ifndef STEREO_CAMERA_RECORDER
#define STEREO_CAMERA_RECORDER

#include <sys/stat.h>
#include <stereo_camera_calibration/topic_names.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>

namespace stereo_camera_calibration
{
    class StereoCameraRecorder
    {
    public:
        StereoCameraRecorder(ros::NodeHandle &n, ros::NodeHandle &p);
        ~StereoCameraRecorder();

    private:
        ros::NodeHandle nh;
        ros::NodeHandle pn;

        ros::ServiceServer startVideoService;
        ros::ServiceServer stopVideoService;
        ros::ServiceServer takeCaptureService;
        
        ros::Subscriber sub_cam_1_image;
        ros::Subscriber sub_cam_2_image;

        sensor_msgs::Image::ConstPtr camera_msg_1;
        sensor_msgs::Image::ConstPtr camera_msg_2;

        ros::Timer timer;
        cv::VideoWriter video_writer_1;
        cv::VideoWriter video_writer_2;

        std::string video_path_1;
        std::string video_path_2;
        std::string images_path_1;
        std::string images_path_2;
        int cam_1_width;
        int cam_2_width;
        int cam_1_height;
        int cam_2_height;
        double fps;

        bool isRecording = false;

        void rosMsgToCvMat(const sensor_msgs::Image::ConstPtr &msg, cv::Mat &mat);
        void camera1ImageCb(const sensor_msgs::Image::ConstPtr &msg);
        void camera2ImageCb(const sensor_msgs::Image::ConstPtr &msg);
        bool startVideoRecordingCb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
        bool stopVideoRecordingCb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
        bool takeCaptureCb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
        void recordVideoCb(const ros::TimerEvent &);

        void loadParameters();
        void getStringParameter(const char* name, std::string& output, bool is_a_file = false);
        void getIntParameter(const char* name, int& output);
        void getDoubleParameter(const char* name, double& output);
        void fileExists(const std::string &file);
    };
} // namespace

#endif