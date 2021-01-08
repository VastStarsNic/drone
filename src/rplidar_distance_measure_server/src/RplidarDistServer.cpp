#include "ros/ros.h"
#include "RplidarDist.hpp"

int main(int argc, char **argv)
{
    ros::init(argc,argv,"RplidarDistServer");
    ros::NodeHandle nh;
    ros::NodeHandle ns("~");

    RplidarDistNode DistNode(&nh,&ns);
    ros::spin();
    return 0;
}