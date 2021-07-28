#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "lcwd_rplidar_measure/RplidarPassFrame.hpp"
#include <map>
#include <cmath>
#include <vector>
#include <iostream>
#include <algorithm>
#define pi 3.1415926
RplidarPassFrame::RplidarPassFrame(ros::NodeHandle* nh,ros::NodeHandle* ns):nh_(*nh),ns_(*ns),
server_(nh_,"rplidar_pass_frame",boost::bind(&RplidarPassFrame::execute,this,boost::placeholders::_1),true)
{
    ns_.param<std::string>("subscribe_rplidar_topic",subscribe_rplidar_topic_,"/scan_angle_filtered");
    ns_.param<std::string>("pole_distance_topic",pole_distance_topic_,"pole_distance");

    nh_.param<double>("/RplidarDistServer/rplidar_pass_frame/max_tolerance_angle", max_tolerance_angle_, 3);
    nh_.param<double>("/RplidarDistServer/rplidar_pass_frame/MAXCDF", MAXCDF_, 0.5);
    nh_.param<double>("/RplidarDistServer/rplidar_pass_frame/MAXPDF", MAXPDF_, 0.2);
    nh_.param<double>("/RplidarDistServer/rplidar_pass_frame/MINPDF", MINPDF_, 0);
    nh_.param<int>("/RplidarDistServer/rplidar_pass_frame/group_margin", group_margin_, 5);
    nh_.param<double>("/RplidarDistServer/rplidar_pass_frame/PID/dist/p", dist_p_, 1);
    nh_.param<double>("/RplidarDistServer/rplidar_pass_frame/PID/dist/i", dist_i_, 0);
    nh_.param<double>("/RplidarDistServer/rplidar_pass_frame/PID/dist/d", dist_d_, 0);
    nh_.param<double>("/RplidarDistServer/rplidar_pass_frame/limit_x_speed", limit_x_speed_, 0.1);
    nh_.param<double>("/RplidarDistServer/rplidar_pass_frame/limit_y_speed", limit_y_speed_, 0.2);

    ros_rplidar_subscriber_ = nh_.subscribe(subscribe_rplidar_topic_,1,&RplidarPassFrame::Rplidar_Callback,this);
    pole_distance_publisher_ = nh_.advertise<sensor_msgs::LaserScan>(pole_distance_topic_, 5, true);
    dynamic_func_ = boost::bind(&RplidarPassFrame::dynamic_callback, this, boost::placeholders::_1, boost::placeholders::_2);
    dynamic_server_.setCallback(dynamic_func_);
	fill_ = false;
    //dist_ = set_dist_;
    //angle_ = 0;
    
    pid_range_.PID_init(0,dist_p_,dist_i_,dist_d_);

    server_.start();
}

RplidarPassFrame::~RplidarPassFrame()
{ 
    nh_.~NodeHandle();
    ns_.~NodeHandle();
}

void RplidarPassFrame::Rplidar_Callback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    scan_ = scan;
}

void RplidarPassFrame::sort_and_deduplication(vector<int> &c)
{
    sort(c.begin(),c.end());
    auto new_end=unique(c.begin(),c.end());
    c.erase(new_end,c.end());
}
// add struct in hpp
void RplidarPassFrame::rplidar_pole_distance(vector<float> vec_a,float &dist_left,float &angle_left,float &dist_right,float &angle_right,float minAngle,float increment)
{
    map<int,vector<Point>> distance;
    int count=0;
    float pole_distance=0;
    vector<int> key_vec;
    //put all vector data in map,which means a sparse distgram
    for(int i=0;i!=vec_a.size();i++)
    {
        auto dist_i=vec_a.at(i);
        if(std::isnan(dist_i) || std::isinf(dist_i) || dist_i>2.0)
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
            //cout<<group.size()<<' ';
            PDF.push_back(group.size()*1.0/count);
            mean_dist.push_back(mean_dist_/group.size());
            mean_angle.push_back(mean_angle_/group.size());
        }
    }
    float CDF=0;
//	for(auto i: PDF)
//	ROS_INFO("%f",i);
    Pole_position probable_left={0,0,0},probable_right={0,0,0};
    for(int i=0;i!=PDF.size();i++)
    {  
        CDF+=PDF.at(i);
        if(CDF>MAXCDF_)
        break;
        if((PDF.at(i)<=MAXPDF_)&&(PDF.at(i)>=MINPDF_))
        {
            if(PDF.at(i)>probable_left.PDF_ || PDF.at(i)>probable_right.PDF_)
            {
                if(probable_left.PDF_<=probable_right.PDF_)
                {
                    probable_left.PDF_=PDF.at(i);
                    probable_left.range_=mean_dist.at(i);
                    probable_left.angle_=mean_angle.at(i);
                }
                else
                {
                    probable_right.PDF_=PDF.at(i);
                    probable_right.range_=mean_dist.at(i);
                    probable_right.angle_=mean_angle.at(i);
                }
            }         
        }
    }
    if(probable_left.angle_>probable_right.angle_)
    {
     //   ROS_INFO("ORIGINAL DIST:LEFT%f,RIGHT%f",probable_left.range_,probable_right.range_);
        Pole_position temp=probable_left;
        probable_left=probable_right;
        probable_right=temp;
    }
    if(fabs(probable_left.PDF_) < 1e-5 || fabs(probable_right.PDF_)< 1e-5)
    {
        ROS_INFO("lost");
        lost_count_++;
	    return;
    }
	else lost_count_=0;
    if(vector_pole_left_.size()<30 || vector_pole_right_.size()<30)
    {
        vector_pole_left_.push_back(probable_left);
        vector_pole_right_.push_back(probable_right);
	fill_ = false;
    }    
    else
    {
	fill_ = true;
        float angle_mean_left=0,angle_mean_right=0;
        angle_mean_left=compute_dist_mean(vector_pole_left_);
        angle_mean_right=compute_angle_mean(vector_pole_right_);
        if(fabs(probable_left.PDF_)<1e-5||fabs(angle_mean_left-probable_left.angle_)>3*compute_angle_var(vector_pole_left_))
        {
            dist_left=compute_dist_mean(vector_pole_left_);
            angle_left=compute_angle_mean(vector_pole_left_);
        }
        else
        {
            dist_left=probable_left.range_;
            angle_left=probable_left.angle_;
        }

        if(fabs(probable_right.PDF_)<1e-5||fabs(angle_mean_right-probable_right.angle_)>3*compute_angle_var(vector_pole_right_))
        {
            dist_right=compute_dist_mean(vector_pole_right_);
            angle_right=compute_angle_mean(vector_pole_right_);
        }
        else
        {
            dist_right=probable_right.range_;
            angle_right=probable_right.angle_;
        }
        ROS_INFO("LEFT_DIST:%f,LEFT_ANGLE:%f",dist_left,angle_left);
        ROS_INFO("RIGHT_DIST:%f,RIGHT_ANGLE:%f",dist_right,angle_right);
	    ROS_INFO(" ");
        vector_pole_left_.erase(vector_pole_left_.begin());
        vector_pole_left_.push_back(probable_left);
        vector_pole_right_.erase(vector_pole_right_.begin());
        vector_pole_right_.push_back(probable_right);
    }
}

void RplidarPassFrame::pid_speed_control()
{
    //need to adjust 
    pid_range_.PID_set(dist_p_,dist_i_,dist_d_);
    speed_range_ = -pid_range_.PID_location(dist_left_-dist_right_);
    //speed_angle_ = pid_angle_.PID_increment(angle_p_);

    speed_range_ = speed_range_ > limit_y_speed_? limit_y_speed_ : speed_range_;
    speed_range_ = speed_range_ < -limit_y_speed_? -limit_y_speed_ : speed_range_;
    
    //speed_angle_ = speed_angle_ > limit_z_angle_speed_? limit_z_angle_speed_ : speed_angle_;
    //speed_angle_ = speed_angle_ < -limit_z_angle_speed_? -limit_z_angle_speed_ : speed_angle_;    
    //20° as a threshold
    
    if(fabs(dist_left_-dist_right_)>0.15)
    {
        speed_.angular.z = 0;
        speed_.linear.x = 0;
        speed_.linear.y = speed_range_;
        
    }
    else
    {
        speed_.angular.z = 0;
        speed_.linear.x = 0.15;
        //this is our set speed on y-axis 
        speed_.linear.y = speed_range_;
    }
}

void RplidarPassFrame::dynamic_callback(rplidar_distance::rplidar_distConfig &config, uint32_t level)
{
    MAXCDF_ = config.MAXCDF;
    MAXPDF_ = config.MAXPDF;
    MINPDF_ = config.MINPDF;
    dist_p_ = config.kp;
    dist_i_ = config.ki;
    dist_d_ = config.kd;
}

float RplidarPassFrame::cos_theorem(float a,float b,float c)
{
    return acos((b*b+c*c-a*a)/(2*b*c));
}

float RplidarPassFrame::get_target_theta()
{
    float theta_1=0,theta_2=0,theta_3=0,target_theta_left=0;
    //vector<float> vec(begin(scan_->ranges),end(scan_->ranges));
    //rplidar_pole_distance(vec,dist_left_,angle_left_,dist_right_,angle_right_,scan_->angle_min,scan_->angle_increment);
    theta_1=cos_theorem(dist_right_,dist_left_,1);//the third parameter is the distance between two poles
    theta_2=cos_theorem(dist_left_,dist_right_,1);
    theta_3=cos_theorem(1,dist_left_,dist_right_);
    if(theta_1>pi/2 || theta_2>pi/2)
    {
	ROS_INFO("EXIST THETA BIGGER THAN PI/2,THE THETA IS %f",theta_1>theta_2?theta_1:theta_2);
        if(theta_1>pi/2)
            target_theta_left=theta_1-pi/2;
        else
            target_theta_left=-(theta_2-pi/2+theta_3);
    }
    else
    {
	ROS_INFO("NO THETA BIGGER THAN PI/2");
        target_theta_left=-atan2(dist_left_-dist_right_*cos(angle_right_-angle_left_),dist_right_*(angle_right_-angle_left_));
    }
    return target_theta_left;
}
void RplidarPassFrame::execute(const lcwd_rplidar_measure::RplidarMeasureGoalConstPtr& goal)
{
    nh_.param<double>("/RplidarDistServer/rplidar_pass_frame/PID/dist/p", dist_p_, 1);
    nh_.param<double>("/RplidarDistServer/rplidar_pass_frame/PID/dist/i", dist_i_, 0);
    nh_.param<double>("/RplidarDistServer/rplidar_pass_frame/PID/dist/d", dist_d_, 0);
    nh_.param<double>("/RplidarDistServer/rplidar_pass_frame/MAXCDF", MAXCDF_, 1);
    nh_.param<double>("/RplidarDistServer/rplidar_pass_frame/MAXPDF", MAXPDF_, 1);
    nh_.param<double>("/RplidarDistServer/rplidar_pass_frame/MINPDF", MINPDF_, 0);
    ros::Rate rate(30);
    lcwd_rplidar_measure::RplidarMeasureFeedback feedback;
    ROS_INFO("RplidarPassFrame Action Server Execute"); 
    bool no_find = false;
    while(!fill_)
    {
        if(no_find)
            break;
        vector<float> vec(begin(scan_->ranges),end(scan_->ranges));
        rplidar_pole_distance(vec,dist_left_,angle_left_,dist_right_,angle_right_,scan_->angle_min,scan_->angle_increment);
        if(lost_count_ >= 10)
        {
            if(lost_count_ == 10)
                lost_time_ = event_.current_expected.now().toSec();
            lost_target_control(no_find);
        }
        rate.sleep();
        ROS_INFO("fill no end"); 
    }
    float target_theta_left=get_target_theta();

    while(fabs(target_theta_left-angle_left_)>0.1)
    {
	vector<float> vec(begin(scan_->ranges),end(scan_->ranges));
        rplidar_pole_distance(vec,dist_left_,angle_left_,dist_right_,angle_right_,scan_->angle_min,scan_->angle_increment);
        ROS_INFO("adjusting raw");
        if(angle_left_<target_theta_left)
        {
            speed_.linear.x = 0;
            speed_.linear.y = 0;
            speed_.angular.z = 0.15;
        }
        else
        {
            speed_.linear.x = 0;
            speed_.linear.y = 0;
            speed_.angular.z = -0.15;
        }
        feedback.twist = speed_;
        server_.publishFeedback(feedback); 
        rate.sleep(); 
    }

    do
    {
        ROS_INFO("pass frame");
        if(no_find)
            break;
        vector<float> vec(begin(scan_->ranges),end(scan_->ranges));
        rplidar_pole_distance(vec,dist_left_,angle_left_,dist_right_,angle_right_,scan_->angle_min,scan_->angle_increment);
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
    rate.sleep(); 
    }while((angle_left_>-3*pi/4)||(angle_right_<3*pi/4));

    if(no_find)
        ROS_INFO("Fail to find frame");
    else
        ROS_INFO("RplidarPassFrame Execute Finish");
    server_.setSucceeded();
}
double RplidarPassFrame::compute_angle_mean(const vector<Pole_position> &pole_vec)
{
	if(pole_vec.size()==0)
		return 0;
    double sum=0;
    for(auto i:pole_vec)
        sum+=i.angle_;
    return sum/pole_vec.size();
}
double RplidarPassFrame::compute_dist_mean(const vector<Pole_position> &pole_vec)
{
if(pole_vec.size()==0)
		return 0;
    double sum=0;
    for(auto i:pole_vec)
        sum+=i.range_;
    return sum/pole_vec.size();
}
double RplidarPassFrame::compute_angle_var(const vector<Pole_position> &pole_vec)
{
    float var=0,mean=0;
    mean=compute_angle_mean(pole_vec);
    for(auto i:pole_vec)
    {
        var+=pow(i.angle_-mean,2);
    }
    var=sqrt(var/pole_vec.size());
    return var;
}
void RplidarPassFrame::lost_target_control(bool &no_find)
{
	double now_time_ = event_.current_expected.now().toSec();
    if(now_time_ - lost_time_ < 3)
    {
        speed_.linear.x = 0.15;
        speed_.linear.y = 0;
        speed_.angular.z = 0;
    }
    else
    {
        speed_.linear.x = 0;
        speed_.linear.y = 0;
        speed_.angular.z = 0;
        no_find = true;
    }
}
