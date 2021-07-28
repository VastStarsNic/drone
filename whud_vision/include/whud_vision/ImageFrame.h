#pragma once

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <whud_vision/ImageTransmit.h>

/**
 * This is whud_vision node class
 */
namespace whud_vision
{

    class ImageFrame: public nodelet::Nodelet
    {
        public:
            ImageFrame();
            ~ImageFrame();
            void onInit();
        private:
        ros::NodeHandle nh_;
        ImageTransmit *ImageTransmit1;
        ImageTransmit *ImageTransmit2;
        ImageTransmit *ImageTransmit3;

        std::map<std::string,PluginPtr> all_plugin_map_;
        pluginlib::ClassLoader<vision_base::VisionBase> plugin_loader_;

        bool is_blacklist(std::string &pl_name,
                                 std::vector<std::string> &blacklist, 
                                 std::vector<std::string> &whitelist);

        std::vector<int> get_index_without_blacklist(std::vector<std::string>original_plugins,
                                                     std::vector<std::string> &blacklist, 
                                                     std::vector<std::string> &whitelist);

        void set_plugins_without_blacklist(Video* video, std::vector<int>new_index);

        Video plugin_msg(std::string index,ros::NodeHandle nh, 
                         std::vector<std::string>blacklist, std::vector<std::string>whitelist);
    };

}