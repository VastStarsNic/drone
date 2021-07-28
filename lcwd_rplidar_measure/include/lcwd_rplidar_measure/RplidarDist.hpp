#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include<dynamic_reconfigure/server.h>
#include "lcwd_rplidar_measure/rplidar_distConfig.h"
#include "lcwd_rplidar_measure/pid.hpp"
#include <geometry_msgs/Twist.h>
#include <actionlib/server/simple_action_server.h>
#include "lcwd_rplidar_measure/RplidarMeasureAction.h"
#include <tf/transform_listener.h>
#include <queue>
using namespace std;

struct Point{
    float range_;
    float angle_;
};

struct Pole_position{
    float range_;
    float angle_;
    float PDF_;
};
class RplidarMeasure
{
    typedef actionlib::SimpleActionServer<lcwd_rplidar_measure::RplidarMeasureAction> Server;
    public:
        RplidarMeasure(ros::NodeHandle* nh,ros::NodeHandle* ns);
        ~RplidarMeasure();
    private:
        dynamic_reconfigure::Server<rplidar_distance::rplidar_distConfig> dynamic_server_;
        dynamic_reconfigure::Server<rplidar_distance::rplidar_distConfig>::CallbackType dynamic_func_; 
        double MAXCDF_;
        double MAXPDF_,MINPDF_;
        double lost_time_;
        int lost_count_;

	    vector<Pole_position> vector_pole_;	

        ros::NodeHandle nh_;
        ros::NodeHandle ns_;
        ros::TimerEvent event_;
        Server server_;

        Pid_control pid_range_;
        Pid_control pid_angle_;

        float dist_, angle_, speed_range_, speed_angle_;
        geometry_msgs::Twist speed_;

        ros::Subscriber ros_rplidar_subscriber_;
        ros::Publisher pole_distance_publisher_;

        std::string subscribe_rplidar_topic_;
        std::string pole_distance_topic_;
        std::string lidar_target_id_, lidar_source_id_;

        sensor_msgs::LaserScan::ConstPtr scan_;
        tf::TransformListener tf_listener_;
        tf::StampedTransform tf_transform_;

        void sort_and_deduplication(vector<int> &c);
        void Rplidar_Callback(const sensor_msgs::LaserScan::ConstPtr& scan);
        void dynamic_callback(rplidar_distance::rplidar_distConfig &config, uint32_t level);
        void rplidar_pole_distance(vector<float> vec_a,float &dist,float &angle,float minAngle,float increment);
        void pid_speed_control();
        void execute(const lcwd_rplidar_measure::RplidarMeasureGoalConstPtr& goal);
        void lost_target_control(bool &no_find);
        //params
        double max_tolerance_angle_;
        double dist_p_, dist_i_, dist_d_, set_dist_, angle_p_, angle_i_, angle_d_;
        int group_margin_;
        double rotation_angle_;
        double limit_x_speed_, limit_y_speed_, limit_z_angle_speed_;
};
