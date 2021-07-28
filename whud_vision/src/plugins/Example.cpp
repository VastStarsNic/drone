#include "plugins/PluginBase.hpp"
#include <opencv2/opencv.hpp>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

using namespace cv;
using namespace std;

namespace vision_plugins
{
    class Example: public vision_base::VisionBase
    {
        public:
            Example();
            void initialize();
            
            //override interface function of basis class
            void UpdateImage(Mat image);

        private:
            Mat image_now_;
    };

    Example::Example()
    {

    }

    void Example::initialize()
    {
        ROS_INFO("Example Init");
    }

    void Example::UpdateImage(Mat image)
    {
        ROS_INFO("Example:update");
        image_now_ = image.clone();
    }
}

PLUGINLIB_EXPORT_CLASS(vision_plugins::Example, vision_base::VisionBase);