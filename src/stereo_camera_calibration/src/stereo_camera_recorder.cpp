#include <stereo_camera_calibration/stereo_camera_recorder.h>

namespace stereo_camera_calibration
{
    StereoCameraRecorder::StereoCameraRecorder(ros::NodeHandle &n, ros::NodeHandle &p)
    {
        this->nh = n;
        this->pn = p;
        this->loadParameters();
        ROS_DEBUG("parameters ok");

        this->startVideoService = this->pn.advertiseService(SERVICE_START_VIDEO_RECORDING, &StereoCameraRecorder::startVideoRecordingCb, this);
        this->stopVideoService = this->pn.advertiseService(SERVICE_STOP_VIDEO_RECORDING, &StereoCameraRecorder::stopVideoRecordingCb, this);
        this->takeCaptureService = this->pn.advertiseService(SERVICE_TAKE_CAPTURE, &StereoCameraRecorder::takeCaptureCb, this);
        this->timer = this->nh.createTimer(ros::Duration(1. / this->fps), &StereoCameraRecorder::recordVideoCb, this, false, false);
        ROS_DEBUG("services & timer ok");

        this->sub_cam_1_image = this->nh.subscribe<sensor_msgs::Image>(TOPIC_CAM_1_IMG, 1, &StereoCameraRecorder::camera1ImageCb, this);
        this->sub_cam_2_image = this->nh.subscribe<sensor_msgs::Image>(TOPIC_CAM_2_IMG, 1, &StereoCameraRecorder::camera2ImageCb, this);
        ROS_DEBUG("subscriptions ok");

        ROS_INFO("Waiting for services calls...");
    }

    StereoCameraRecorder::~StereoCameraRecorder()
    {
        this->isRecording = false;
        if (this->video_writer_1.isOpened())
            this->video_writer_1.release();
        if (this->video_writer_2.isOpened())
            this->video_writer_2.release();
    }

    bool StereoCameraRecorder::startVideoRecordingCb(
        std_srvs::Trigger::Request &req,
        std_srvs::Trigger::Response &res)
    {
        if (!this->isRecording)
        {
            ROS_INFO("Video recording started");

            this->video_writer_1.open(this->video_path_1, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), this->fps, cv::Size(this->cam_1_width, this->cam_1_height), true);
            this->video_writer_2.open(this->video_path_2, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), this->fps, cv::Size(this->cam_2_width, this->cam_2_height), true);

            this->isRecording = true;
            this->timer.start();

            res.success = true;
            res.message = MSG_START_VIDEO_RECORDING;
        }
        else
        {
            res.success = false;
            res.message = MSG_RECORDING;
        }

        return true;
    }

    bool StereoCameraRecorder::stopVideoRecordingCb(
        std_srvs::Trigger::Request &req,
        std_srvs::Trigger::Response &res)
    {
        if (this->isRecording)
        {
            ROS_INFO("Video recording stoped");
            this->timer.stop();
            this->isRecording = false;

            this->video_writer_1.release();
            this->video_writer_2.release();

            res.success = true;
            res.message = MSG_STOP_VIDEO_RECORDING;
        }
        else
        {
            res.success = false;
            res.message = MSG_NOT_RECORDING;
        }

        return true;
    }

    bool StereoCameraRecorder::takeCaptureCb(
        std_srvs::Trigger::Request &req,
        std_srvs::Trigger::Response &res)
    {
        if (this->isRecording)
        {
            res.success = false;
            res.message = MSG_RECORDING;
        }
        else
        {
            if (this->camera_msg_1 != nullptr && this->camera_msg_2 != nullptr)
            {
                static unsigned int index = 100;
                cv::Mat img1;
                cv::Mat img2;

                this->rosMsgToCvMat(this->camera_msg_1, img1);
                this->rosMsgToCvMat(this->camera_msg_2, img2);

                std::string path1 = this->images_path_1 + IMAGE_1_NAME + std::to_string(index) + ".png";
                std::string path2 = this->images_path_2 + IMAGE_2_NAME + std::to_string(index) + ".png";

                cv::imwrite(path1, img1);
                cv::imwrite(path2, img2);
                ++index;

                ROS_INFO("Capture taken in %s", path1.c_str());
                ROS_INFO("Capture taken in %s", path2.c_str());

                res.success = true;
                res.message = MSG_CAPTURE_TAKEN;
            }
            else
            {
                res.success = false;
                res.message = MSG_WRONG_IMAGE_SIZE;
            }
        }

        return true;
    }

    void StereoCameraRecorder::recordVideoCb(const ros::TimerEvent &)
    {
        if (this->isRecording && this->camera_msg_1 != nullptr && this->camera_msg_2 != nullptr)
        {
            // ROS_DEBUG("recording...");
            cv::Mat img1;
            cv::Mat img2;

            this->rosMsgToCvMat(this->camera_msg_1, img1);
            this->rosMsgToCvMat(this->camera_msg_2, img2);

            this->video_writer_1.write(img1);
            this->video_writer_2.write(img2);
        }
    }

    void StereoCameraRecorder::camera1ImageCb(const sensor_msgs::Image::ConstPtr &msg)
    {
        // ROS_DEBUG("cam1 callback");
        this->camera_msg_1 = msg;
    }

    void StereoCameraRecorder::camera2ImageCb(const sensor_msgs::Image::ConstPtr &msg)
    {
        // ROS_DEBUG("cam2 callback");
        this->camera_msg_2 = msg;
    }

    void StereoCameraRecorder::rosMsgToCvMat(const sensor_msgs::Image::ConstPtr &msg, cv::Mat &mat)
    {
        // ROS_DEBUG("rosMsgToCvImg");
        try
        {
            cv_bridge::CvImagePtr img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            mat = img->image;
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            exit(1);
        }
    }

    void StereoCameraRecorder::loadParameters()
    {
        this->getStringParameter(PARAM_VIDEO_PATH_1, this->video_path_1);
        this->getStringParameter(PARAM_VIDEO_PATH_2, this->video_path_2);
        this->getStringParameter(PARAM_IMAGES_PATH_1, this->images_path_1);
        this->getStringParameter(PARAM_IMAGES_PATH_2, this->images_path_2);
        this->getIntParameter(PARAM_CAM_1_WIDTH, this->cam_1_width);
        this->getIntParameter(PARAM_CAM_2_WIDTH, this->cam_2_width);
        this->getIntParameter(PARAM_CAM_2_HEIGHT, this->cam_2_height);
        this->getIntParameter(PARAM_CAM_1_HEIGHT, this->cam_1_height);
        this->getDoubleParameter(PARAM_FPS, this->fps);
    }

    void StereoCameraRecorder::getStringParameter(const char *name, std::string &output, bool is_a_file)
    {
        ROS_ASSERT(this->nh.getParam(name, output));
        if (is_a_file)
        {
            this->fileExists(output);
        }
        ROS_DEBUG("%s = %s", name, output.c_str());
    }

    void StereoCameraRecorder::getIntParameter(const char *name, int &output)
    {
        ROS_ASSERT(this->nh.getParam(name, output));
        ROS_DEBUG("%s = %d", name, output);
    }

    void StereoCameraRecorder::getDoubleParameter(const char *name, double &output)
    {
        XmlRpc::XmlRpcValue tmp;
        ROS_ASSERT(this->nh.getParam(name, tmp));
        ROS_ASSERT(tmp.getType() == XmlRpc::XmlRpcValue::TypeDouble);
        output = tmp;
        ROS_DEBUG("%s = %f", name, output);
    }

    void StereoCameraRecorder::fileExists(const std::string &file)
    {
        struct stat buffer;
        ROS_ASSERT(stat(file.c_str(), &buffer) == 0);
    }

} // namespace stereo_camera_calibration