
#pragma once

#include <ros/ros.h>
/**
 * The plugin basis class header file
 */
#include <plugins/PluginBase.hpp>

#include <pluginlib/class_loader.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>


namespace whud_vision
{
    using PluginPtr = boost::shared_ptr<vision_base::VisionBase>;
    struct Video
    {
        std::string subscriber_topic;
        int video_frequency;
        std::vector<std::string> plugins;
        std::vector<int> plugins_frequency;
    };

    class ImageTransmit
    {
        public:            
            ImageTransmit(Video video, ros::NodeHandle* nh, std::map<std::string,PluginPtr> all_plugin_map);
            ~ImageTransmit(){}

        private:
            //Save plugins' message.
            ros::NodeHandle n_image_;  
            //image_transport::ImageTransport n_image_;
            int video_frequency_ = 30;            
            ros::Subscriber video_sub_;
            //image_transport::Subscriber video_sub_;
            std::vector<std::string> plugins_;
            std::vector<int> plugins_frequency_;

            //plugin loader and plugin vector
            std::map<std::string,PluginPtr> plugin_map_;


            int count = 0;        
            void image_callback(const sensor_msgs::ImageConstPtr& msg);
    };    
}
