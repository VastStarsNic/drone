#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "RplidarDist.hpp"

RplidarDistNode::RplidarDistNode(ros::NodeHandle* nh,ros::NodeHandle* ns):_nh(*nh),_ns(*ns)
{
    _ns.param<std::string>("subscribe_rplidar_topic",_subscribe_rplidar_topic,"/scan_filtered");
    _ns.param<std::string>("pole_distance_topic",_pole_distance_topic,"pole_distance");
    SubscriberInit();
    PublisherInit();
}

RplidarDistNode::~RplidarDistNode()
{ 
    _nh.~NodeHandle();
    _ns.~NodeHandle();
}

void RplidarDistNode::SubscriberInit()
{
    _ros_rplidar_subscriber = _nh.subscribe(_subscribe_rplidar_topic,1,&RplidarDistNode::Rplidar_Callback,this);
}

void RplidarDistNode::PublisherInit()
{
    _pole_distance_publisher = _nh.advertise<sensor_msgs::LaserScan>(_pole_distance_topic, 5, true);
}

void RplidarDistNode::Rplidar_Callback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    vector<float> vec(begin(scan->ranges),end(scan->ranges));
    rp_frame rpftr = rp_frame(vec, scan->angle_min, scan->angle_increment);
    float dist=rpftr.get_pole();
    std::cout<<dist<<std::endl;
}