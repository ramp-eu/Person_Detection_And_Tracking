#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <camera_tracker/camera_tracker.h>

namespace camera_tracker
{
    class CameraTrackerNodelet : public nodelet::Nodelet
    {
    public:
        CameraTrackerNodelet(){};
        ~CameraTrackerNodelet(){};

        void onInit(void)
        {
            tfc.reset(new CameraTracker(getMTNodeHandle(), getMTPrivateNodeHandle()));
        }

    private:
        boost::shared_ptr<CameraTracker> tfc;
    };

} // end namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(camera_tracker::CameraTrackerNodelet, nodelet::Nodelet);
