#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <stereo_camera_calibration/mono_camera_recorder.h>

namespace stereo_camera_calibration
{
    class MonoCameraRecorderNodelet : public nodelet::Nodelet
    {
    public:
        MonoCameraRecorderNodelet(){};
        ~MonoCameraRecorderNodelet(){};

        void onInit(void)
        {
            mcr.reset(new MonoCameraRecorder(getMTNodeHandle(), getMTPrivateNodeHandle()));
        }

    private:
        boost::shared_ptr<MonoCameraRecorder> mcr;
    };

} // end namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(stereo_camera_calibration::MonoCameraRecorderNodelet, nodelet::Nodelet);