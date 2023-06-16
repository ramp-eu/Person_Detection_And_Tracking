#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <stereo_camera_calibration/stereo_camera_recorder.h>

namespace stereo_camera_calibration
{
    class StereoCameraRecorderNodelet : public nodelet::Nodelet
    {
    public:
        StereoCameraRecorderNodelet(){};
        ~StereoCameraRecorderNodelet(){};

        void onInit(void)
        {
            scr.reset(new StereoCameraRecorder(getMTNodeHandle(), getMTPrivateNodeHandle()));
        }

    private:
        boost::shared_ptr<StereoCameraRecorder> scr;
    };

} // end namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(stereo_camera_calibration::StereoCameraRecorderNodelet, nodelet::Nodelet);