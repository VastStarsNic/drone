#include "ros/ros.h"
#include "lcwd_rplidar_measure/RplidarDist.hpp"
#include "lcwd_rplidar_measure/RplidarPassFrame.hpp"
int main(int argc, char **argv)
{
    ros::init(argc,argv,"RplidarDistServer");
    ros::NodeHandle nh;
    ros::NodeHandle ns("~");

    //RplidarMeasure DistNode(&nh,&ns);
    RplidarPassFrame PassNode(&nh,&ns);
    ros::spin();
    return 0;
}