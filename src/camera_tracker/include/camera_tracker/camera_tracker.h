#ifndef CAMERA_TRACKER_H
#define CAMERA_TRACKER_H

#include <opencv2/core/ocl.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/utility.hpp>

#include <sys/stat.h>
#include <iostream>
#include <fstream>
#include <random>
#include <set>
#include <cmath>
#include <vector>
#include <time.h>
#include <string>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <mapupdates_msgs/NewObstacles.h>

#include <camera_tracker/Hungarian.h>
#include <camera_tracker/TrackerClass.h>
#include <camera_tracker/Tracking.h>
#include <camera_tracker/Detection.h>

#define STOP 0
#define INIT 1
#define READING 2
#define DETECT 3
#define TRACK 4
#define PUBLISH 5

#define SHOW_IMAGES

namespace camera_tracker
{
    class CameraTracker
    {
    public:
        CameraTracker(ros::NodeHandle &n, ros::NodeHandle &p);
        ~CameraTracker();

    private:
        ros::NodeHandle nh;
        ros::NodeHandle pn;
        ros::Publisher pub_worker_obstacles;
        ros::Publisher pub_worker_position;
        ros::Publisher pub_traking_image_1;
        ros::Publisher pub_traking_image_2;
        ros::Subscriber sub_cam_1_image;
        ros::Subscriber sub_cam_2_image;
        ros::Timer timer;

        mapupdates_msgs::NewObstacles newobstacles_msg;
        geometry_msgs::PoseArray worker_positions_msg;

        sensor_msgs::Image::ConstPtr camera_msg_1;
        sensor_msgs::Image::ConstPtr camera_msg_2;
        cv_bridge::CvImagePtr input_image_1;
        cv_bridge::CvImagePtr input_image_2;
        cv_bridge::CvImagePtr output_image_1;
        cv_bridge::CvImagePtr output_image_2;

        ////////////////////////////////////////////////////////
        /*++++++++++++++++++++ YOLO v3 +++++++++++++++++++++++*/
        ////////////////////////////////////////////////////////
        cv::dnn::Net net_yolo;
        float confThreshold = 0.7;         // Confidence threshold 0.7
        float nmsThreshold = 0.7;          // Non-maximum suppression threshold 0.45
        int inpWidth = 416;                // Width of network's input image
        int inpHeight = 416 * 1088 / 2048; // Height of network's input image
        vector<string> classes;
        // Params
        std::string cam_calibration_file;
        int cam1_id, cam2_id;
        std::string yolo_classes_file, yolo_model_file, yolo_weights_file;
        double exp_t1, exp_t2, fps, sample_time_limit, timer_rate;

        volatile int stop = 0;
        double start, finish;
        int no_frames = 0;
        int state;
    
        vector<Detection *> detections_cam1, detections_cam2;
        cv::Mat K_cam1, dist_coeffs_cam1, K_cam2, dist_coeffs_cam2, R, T, P1, P2, R1, R2, Q, F;
        int img_width_cam1, img_height_cam1, img_width_cam2, img_height_cam2;
        Mat map1_cam1, map2_cam1, map1_cam2, map2_cam2;
        Tracking tracking_node;

        bool first = true;
        
        double get_current_time(void);
        void shutdown_tracker(int error);
        void postprocess(cv::Mat &frame, const vector<cv::Mat> &outs, vector<Detection *> &detections);
        vector<string> getOutputsNames(const cv::dnn::Net &net);
        void peopleDetection(cv::dnn::Net &net_, cv::Mat &img, int inpWidth, int inpHeight, vector<Detection *> &detections);
        float distancePointLine(cv::Point2f p, cv::Point3f line);
        void stereo_matching(vector<Detection *> &detections_cam1, vector<Detection *> &detections_cam2,
                             Mat &F, vector<int> &assignment, vector<vector<double>> &cost_matrix, Mat img1, Mat img2);
        inline void checkIfFileExists(const std::string& file) const;
        void camera1ImageCb(const sensor_msgs::Image::ConstPtr &msg);
        void camera2ImageCb(const sensor_msgs::Image::ConstPtr &msg);
        void rosMsgToCvImg(const sensor_msgs::Image::ConstPtr& msg, cv_bridge::CvImagePtr& img);
        void runStateMachineCb(const ros::TimerEvent&);
        void publishOutputImages();
        void publishInputImages();
        void install_params();
        void loadCalibrationParameters();
        void run();
    };
} // end namespace

#endif
