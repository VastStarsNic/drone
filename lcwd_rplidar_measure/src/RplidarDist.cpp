#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "lcwd_rplidar_measure/RplidarDist.hpp"
#include <map>
#include <cmath>
#include <vector>
#include <iostream>
#include <algorithm>

RplidarMeasure::RplidarMeasure(ros::NodeHandle* nh,ros::NodeHandle* ns):nh_(*nh),ns_(*ns),
server_(nh_,"rplidar_measure",boost::bind(&RplidarMeasure::execute,this,boost::placeholders::_1),true)
{
    ns_.param<std::string>("subscribe_rplidar_topic",subscribe_rplidar_topic_,"/scan_angle_filtered");
    ns_.param<std::string>("pole_distance_topic",pole_distance_topic_,"pole_distance");

    nh_.param<double>("/RplidarDistServer/rplidar_measure/max_tolerance_angle", max_tolerance_angle_, 3);
    nh_.param<double>("/RplidarDistServer/rplidar_measure/MAXCDF", MAXCDF_, 0.5);
    nh_.param<double>("/RplidarDistServer/rplidar_measure/MAXPDF", MAXPDF_, 0.2);
    nh_.param<double>("/RplidarDistServer/rplidar_measure/MINPDF", MINPDF_, 0);
    nh_.param<int>("/RplidarDistServer/rplidar_measure/group_margin", group_margin_, 5);
    nh_.param<double>("/RplidarDistServer/rplidar_measure/PID/dist/p", dist_p_, 1);
    nh_.param<double>("/RplidarDistServer/rplidar_measure/PID/dist/i", dist_i_, 0);
    nh_.param<double>("/RplidarDistServer/rplidar_measure/PID/dist/d", dist_d_, 0);
    nh_.param<double>("/RplidarDistServer/rplidar_measure/PID/dist/set_dist", set_dist_, 1);
    nh_.param<double>("/RplidarDistServer/rplidar_measure/PID/angle/p", angle_p_, 1);
    nh_.param<double>("/RplidarDistServer/rplidar_measure/PID/angle/i", angle_i_, 0);
    nh_.param<double>("/RplidarDistServer/rplidar_measure/PID/angle/d", angle_d_, 0);    
    nh_.param<std::string>("/RplidarDistServer/rplidar_measure/lidar_target", lidar_target_id_, "map");
    nh_.param<std::string>("/RplidarDistServer/rplidar_measure/lidar_source", lidar_source_id_,"track_link");
    nh_.param<double>("/RplidarDistServer/rplidar_measure/rotation_angle", rotation_angle_,1.57);
    nh_.param<double>("/RplidarDistServer/rplidar_measure/limit_x_speed", limit_x_speed_, 0.2);
    nh_.param<double>("/RplidarDistServer/rplidar_measure/limit_y_speed", limit_y_speed_, 0.1);
    nh_.param<double>("/RplidarDistServer/rplidar_measure/limit_z_angle_speed", limit_z_angle_speed_, 0.3);   

    ros_rplidar_subscriber_ = nh_.subscribe(subscribe_rplidar_topic_,1,&RplidarMeasure::Rplidar_Callback,this);
    pole_distance_publisher_ = nh_.advertise<sensor_msgs::LaserScan>(pole_distance_topic_, 5, true);
    dynamic_func_ = boost::bind(&RplidarMeasure::dynamic_callback, this, boost::placeholders::_1, boost::placeholders::_2);
    dynamic_server_.setCallback(dynamic_func_);
    dist_ = set_dist_;
    angle_ = 0;
    pid_range_.PID_init(set_dist_,dist_p_,dist_i_,dist_d_);
    pid_angle_.PID_init(0,angle_p_,angle_i_,angle_d_);

    server_.start();
}

RplidarMeasure::~RplidarMeasure()
{ 
    nh_.~NodeHandle();
    ns_.~NodeHandle();
}

void RplidarMeasure::Rplidar_Callback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    scan_ = scan;
}

void RplidarMeasure::sort_and_deduplication(vector<int> &c)
{
    sort(c.begin(),c.end());
    auto new_end=unique(c.begin(),c.end());
    c.erase(new_end,c.end());
}
// add struct in hpp
void RplidarMeasure::rplidar_pole_distance(vector<float> vec_a,float &dist,float &angle,float minAngle,float increment)
{
    map<int,vector<Point>> distance;
    int count=0;
    float pole_distance=0;
    vector<int> key_vec;
    //put all vector data in map,which means a sparse distgram
    for(int i=0;i!=vec_a.size();i++)
    {
        auto dist_i=vec_a.at(i);
        if(std::isinf(dist_i) || dist_i>2.0)
        {
            //ROS_INFO("INF DETECT");
            continue;
        }
        //margin
        int index=ceil(dist_i*(100/group_margin_));
        if(distance.find(index)==distance.cend())
        {
            vector<Point> vec;
            vec.push_back({dist_i,minAngle+increment*i});
            distance.insert(pair<int,vector<Point>>({index,vec}));
            key_vec.push_back(index);
        }
        else
        {
            auto &vec=distance.find(index)->second;
            vec.push_back({dist_i,minAngle+increment*i});
        }
        count++;
    }
    sort_and_deduplication(key_vec);
    vector<float> PDF,mean_dist,mean_angle;
    int count_test=0,count_test1=0;

    for(auto iter=distance.begin(); iter!=distance.end();)
    {  
        decltype(iter) iter_pre;
        vector<Point> vec_each_group;
        do
        {
            auto vec_i=distance[iter->first];
            vec_each_group.insert(vec_each_group.end(),vec_i.begin(),vec_i.end());
            iter_pre = iter;
            iter++;
        } while (iter!=distance.end()&&(iter->first-iter_pre->first==1));
        count_test+=vec_each_group.size();
        while(vec_each_group.size()!=0)
        {
            vector<Point> group;
            group.push_back(vec_each_group.at(0));
            vec_each_group.erase(vec_each_group.begin());
            for(auto i=vec_each_group.begin();i!=vec_each_group.end();)
            {
                bool if_add=false;
                for(auto j=group.begin();j!=group.end();j++)
                {
                    if(fabs(j->angle_-i->angle_)<3.14/180*max_tolerance_angle_)// threshold 3°
                    {
                        group.push_back(*i);
                        i=vec_each_group.erase(i);
                        if_add=true;
                        break;
                    }
                }
                if(!if_add)
                    i++;
            }
            float mean_dist_=0,mean_angle_=0;
            for(auto i=group.begin();i!=group.end();i++)
            {
                mean_dist_+=i->range_;
                mean_angle_+=i->angle_;
            }
            PDF.push_back(group.size()*1.0/count);
            mean_dist.push_back(mean_dist_/group.size());
            mean_angle.push_back(mean_angle_/group.size());
        }
    }
    float max_pdf=0,CDF=0;
    Pole_position probable_1st={0,0,0},probable_2nd={0,0,0};
    for(int i=0;i!=PDF.size();i++)
    {  
        CDF+=PDF.at(i);
        if(CDF>MAXCDF_)
        break;
        if((PDF.at(i)<=MAXPDF_)&&(PDF.at(i)>=MINPDF_)&&(PDF.at(i)>max_pdf))
        {
            lost_count_ = 0;
            if(PDF.at(i)>probable_1st.PDF_)
            {
                max_pdf=PDF.at(i);
                probable_1st.PDF_=PDF.at(i);
                probable_1st.range_=mean_dist.at(i);
                probable_1st.angle_=mean_angle.at(i);
            }
            else if(PDF.at(i)>probable_2nd.PDF_)
            {
                probable_2nd.PDF_=PDF.at(i);
                probable_2nd.range_=mean_dist.at(i);
                probable_2nd.angle_=mean_angle.at(i);
            }
        }
    }
    if(fabs(max_pdf) < 1e-5)
    {
        ROS_INFO("lost");
        lost_count_++;
	    return;
    }

    if(vector_pole_.size()<5)
        vector_pole_.push_back(probable_1st);
    else
    {
        float queue_angle_mean=0;
        for(auto i:vector_pole_)
            queue_angle_mean+=i.angle_;
        queue_angle_mean/=vector_pole_.size();
        if(fabs(probable_2nd.PDF_)<1e-5||fabs(queue_angle_mean-probable_1st.angle_)<fabs(queue_angle_mean-probable_2nd.angle_))
        {
            dist=probable_1st.range_;
            angle=probable_1st.angle_;
        }
        else
        {
            dist=probable_2nd.range_;
            angle=probable_2nd.angle_;
        }
        ROS_INFO("DIST:%f,ANGLE:%f",dist,angle);
        vector_pole_.erase(vector_pole_.begin());
        vector_pole_.push_back(probable_1st);
    }
}

void RplidarMeasure::pid_speed_control()
{
    //need to adjust 
    speed_range_ = -pid_range_.PID_increment(dist_);
    speed_angle_ = pid_angle_.PID_increment(angle_);

    speed_range_ = speed_range_ > limit_x_speed_? limit_x_speed_ : speed_range_;
    speed_range_ = speed_range_ < -limit_x_speed_? -limit_x_speed_ : speed_range_;
    
    speed_angle_ = speed_angle_ > limit_z_angle_speed_? limit_z_angle_speed_ : speed_angle_;
    speed_angle_ = speed_angle_ < -limit_z_angle_speed_? -limit_z_angle_speed_ : speed_angle_;    
    //20° as a threshold
    
    if((angle_ > 3.14/180*20) || (angle_ < -3.14/180*20))
    {
        speed_.angular.z = speed_angle_;
        speed_.linear.x = 0;
        speed_.linear.y = 0;
        
    }
    else
    {
        speed_.angular.z = speed_angle_;
        speed_.linear.x = speed_range_;
        //this is our set speed on y-axis 
        speed_.linear.y = -0.1;
    }

}

void RplidarMeasure::dynamic_callback(rplidar_distance::rplidar_distConfig &config, uint32_t level)
{
    MAXCDF_ = config.MAXCDF;
    MAXPDF_ = config.MAXPDF;
    MINPDF_ = config.MINPDF;
}

void RplidarMeasure::execute(const lcwd_rplidar_measure::RplidarMeasureGoalConstPtr& goal)
{
    nh_.param<double>("/RplidarDistServer/rplidar_measure/MAXCDF", MAXCDF_, 0.5);
    nh_.param<double>("/RplidarDistServer/rplidar_measure/MAXPDF", MAXPDF_, 0.2);
    nh_.param<double>("/RplidarDistServer/rplidar_measure/MINPDF", MINPDF_, 0);
    ros::Rate rate(30);
    lcwd_rplidar_measure::RplidarMeasureFeedback feedback;
    ROS_INFO("RplidarMeasure Action Server Execute"); 
    double roll,pitch,yaw;
    double delta_yaw;
    bool no_find = false;
    while(abs(delta_yaw)<rotation_angle_)
    {
        if(no_find)
            break;
        try 
        {
            if(tf_listener_.canTransform("/" + lidar_target_id_,"/" + lidar_source_id_,ros::Time(0))) 
            {
                tf_listener_.lookupTransform("/" + lidar_target_id_,"/" + lidar_source_id_,ros::Time(0),tf_transform_);
                tf::Quaternion quat = tf_transform_.getRotation();
                tf::Matrix3x3(quat).getRPY(roll,pitch,yaw);   
                delta_yaw = yaw;
            }
            vector<float> vec(begin(scan_->ranges),end(scan_->ranges));
            rplidar_pole_distance(vec,dist_,angle_,scan_->angle_min,scan_->angle_increment);
	    if(lost_count_ >= 10)
	    {
		    if(lost_count_ == 10)
                lost_time_ = event_.current_expected.now().toSec();
		    lost_target_control(no_find);
        }
	    else
        {
            pid_speed_control();
        }
	    feedback.twist = speed_;
            server_.publishFeedback(feedback);
        }
        catch(tf::LookupException &ex)
        {
            ROS_WARN("%s", ex.what());
        }      
        rate.sleep();
    }   
    if(no_find)
        ROS_INFO("Fail to find pole");
    else
        ROS_INFO("RplidarMeasure Execute Finish");
    server_.setSucceeded();
}
void RplidarMeasure::lost_target_control(bool &no_find)
{
	double now_time_ = event_.current_expected.now().toSec();
    if(now_time_ - lost_time_ < 5)
    {
        speed_.linear.x = 0;
        speed_.linear.y = 0;
        speed_.angular.z = 0.3;
    }
    else if(now_time_ - lost_time_ > 5 && now_time_ - lost_time_ < 15)
    {
        speed_.linear.x = 0;
        speed_.linear.y = 0;
        speed_.angular.z = -0.3;
    }
    else
    {
        speed_.linear.x = 0;
        speed_.linear.y = 0;
        speed_.angular.z = 0;
        no_find = true;
    }
}
