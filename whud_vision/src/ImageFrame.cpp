#include <whud_vision/ImageFrame.h>
#include <std_msgs/String.h>

using namespace whud_vision;

ImageFrame::ImageFrame():plugin_loader_("whud_vision", "vision_base::VisionBase")
{
    ROS_INFO("ImageFrame Start");
}

ImageFrame::~ImageFrame()
{
    delete ImageTransmit1;
    delete ImageTransmit2;
    delete ImageTransmit3;
    ROS_INFO("ImageFrame Close");
}

void ImageFrame::onInit()
{
    nh_ = this->getPrivateNodeHandle();
    //load plugins white-and-black list
    ros::NodeHandle ns("~");
    std::vector<std::string> plugin_blacklist, plugin_whitelist;
    ns.getParam("/whud_vision_nodelet/plugin_blacklist", plugin_blacklist);
    ns.getParam("/whud_vision_nodelet/plugin_whitelist", plugin_whitelist);

    //load all plugins
    for(auto &plugin_name: plugin_loader_.getDeclaredClasses())
    {
        all_plugin_map_[plugin_name] = plugin_loader_.createInstance(plugin_name);
        all_plugin_map_[plugin_name]->initialize();
    }

    //load camera/video set
    ImageTransmit1 = new ImageTransmit(plugin_msg("1",ns,plugin_blacklist,plugin_whitelist),&nh_,all_plugin_map_);
    ImageTransmit2 = new ImageTransmit(plugin_msg("2",ns,plugin_blacklist,plugin_whitelist),&nh_,all_plugin_map_);
    ImageTransmit3 = new ImageTransmit(plugin_msg("3",ns,plugin_blacklist,plugin_whitelist),&nh_,all_plugin_map_);

}

/**
 * Judge if the plugin is in blacklist.
 */ 
bool ImageFrame::is_blacklist(std::string &pl_name, std::vector<std::string> &blacklist, std::vector<std::string> &whitelist)
{
    for(auto &plugin_name_white: whitelist)
    {
        if(pl_name == plugin_name_white)
        {
            for(auto &plugin_name_black: blacklist)
            {
                if(pl_name == plugin_name_black)
                {
                    return true;
                }
            }
            return false;
        }
        return true;
    }
    return true;

}

/**
 * Get the index of plugins in whitelist.
 */ 
std::vector<int> ImageFrame::get_index_without_blacklist(std::vector<std::string>original_plugins,std::vector<std::string> &blacklist, std::vector<std::string> &whitelist)
{
    std::vector<int> new_index;
    for(int i = 0; i < original_plugins.size(); i++)
    {
        if(!is_blacklist(original_plugins.at(i),blacklist,whitelist))
        {
            new_index.push_back(i);
        }
    }
    return new_index;
}

/**
 * Get the plugins and their frequency list which deleted plugins in blacklist.
 */ 
void ImageFrame::set_plugins_without_blacklist(Video* video, std::vector<int>new_index)
{
    std::vector<std::string> new_plugins;
    std::vector<int> new_freq;
    for(int i = 0; i < new_index.size(); i++)
    {
        new_plugins.push_back(video->plugins.at(new_index.at(i)));
        new_freq.push_back(video->plugins_frequency.at(new_index.at(i)));
    }  
    video->plugins = new_plugins;
    video->plugins_frequency = new_freq;
}

Video ImageFrame::plugin_msg(std::string index,ros::NodeHandle nh, std::vector<std::string>blacklist, std::vector<std::string>whitelist)
{
    Video video;
    std::string param_name = "/whud_vision_nodelet/video";
    nh.getParam(param_name+index+"/plugins",video.plugins);
    nh.param<std::string>(param_name+index+"/topic_name",video.subscriber_topic,"/none");
    nh.param<int>(param_name+index+"/videofreq",video.video_frequency,30);
    nh.param<std::vector<int>>(param_name+index+"/plugins_frequency",video.plugins_frequency,{30});
    std::vector<int> whitelist_index = get_index_without_blacklist(video.plugins,blacklist,whitelist);
    set_plugins_without_blacklist(&video, whitelist_index);
    return video;
}

ImageTransmit::ImageTransmit(Video video, ros::NodeHandle* nh,std::map<std::string,PluginPtr> all_plugin_map):n_image_(*nh)
{
    video_frequency_ = video.video_frequency;
    plugins_ = video.plugins;
    plugins_frequency_ = video.plugins_frequency;
    if(video.subscriber_topic != "/none")
    {
        video_sub_ = n_image_.subscribe(video.subscriber_topic, 1, &ImageTransmit::image_callback, this);        
    }
    for(auto plugin_name: plugins_)
    {
        plugin_map_[plugin_name] = all_plugin_map[plugin_name];
    }
}

void ImageTransmit::image_callback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    for(int i = 0; i < plugins_.size(); i++)
    {
        int temp_freq = video_frequency_/plugins_frequency_.at(i);
        if(count%temp_freq == 0)
        {         
            plugin_map_[plugins_.at(i)]->UpdateImage(cv_ptr->image);
        }              
    }
    count += 1;
    count = count % video_frequency_;    
}

PLUGINLIB_EXPORT_CLASS(whud_vision::ImageFrame, nodelet::Nodelet);