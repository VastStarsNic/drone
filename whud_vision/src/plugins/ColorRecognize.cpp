#include <pluginlib/class_list_macros.h>
#include "plugins/PluginBase.hpp"
#include <opencv2/opencv.hpp>
#include "whud_vision/ImageProcessingAction.h"
#include <geometry_msgs/Twist.h>
#include <actionlib/server/simple_action_server.h>
#include <ros/ros.h>
#include "plugins/pid.hpp"
#include <dynamic_reconfigure/server.h>
#include "whud_vision/whud_vision_dynamicConfig.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <fstream>
#include <string>
using namespace cv;
using namespace std;

namespace vision_plugins
{
    typedef actionlib::SimpleActionServer<whud_vision::ImageProcessingAction> Server;
    
    class ColorRecognize: public vision_base::VisionBase
    {
        public:
            ColorRecognize();
           
            //override interface function of basis class
            void initialize();            
            void UpdateImage(Mat image);

        private:
            ros::NodeHandle n_;
            ros::NodeHandle ns_;
            image_transport::ImageTransport it_;
            image_transport::Publisher img_pub_;
            Mat image_now_;
            Mat image_result_;
            Server server_;
            dynamic_reconfigure::Server<whud_vision::whud_vision_dynamicConfig> dynamic_server_;
            dynamic_reconfigure::Server<whud_vision::whud_vision_dynamicConfig>::CallbackType dynamic_func_;
            fstream file_;

            double center_x_ = 640;
            double center_y_ = 360;

            double rate_divide_ = 8.0;

            double speed_x_ = 0;
            double speed_y_ = 0;

            double max_speed_;

            bool flag_hist_;

            Pid_control pid_x_, pid_y_;
            double Kp_x_,Ki_x_, Kd_x_, Kp_y_,Ki_y_, Kd_y_;

            int recognize_freq_;

            int HLow_,HHigh_,SLow_,SHigh_,VLow_,VHigh_;

            int upper_area_, lower_area_;

            void execute(const whud_vision::ImageProcessingGoalConstPtr& goal);
            void color_recognize();
            void dynamic_callback(whud_vision::whud_vision_dynamicConfig &config, uint32_t level);
    };

    ColorRecognize::ColorRecognize():
        ns_("~ColorRecognize"),
        it_(ns_),
        server_(n_,"color_recognize",boost::bind(&ColorRecognize::execute,this,boost::placeholders::_1),true)
    {
        image_now_ = Mat(480,640,CV_8UC3,1);
        img_pub_ = it_.advertise("img_result",1);
        server_.start();
    }

    void ColorRecognize::initialize()
    {
        n_.param<double>("/whud_vision_nodelet/ColorRecognize/max_speed",max_speed_,0.2);
        n_.param<int>("/whud_vision_nodelet/ColorRecognize/recognize_freq",recognize_freq_,10);
        n_.param<int>("/whud_vision_nodelet/ColorRecognize/upper_area",upper_area_,1000);
        n_.param<int>("/whud_vision_nodelet/ColorRecognize/lower_area",lower_area_,100);   
        n_.param<double>("/whud_vision_nodelet/ColorRecognize/Px",Kp_x_,1);
        n_.param<double>("/whud_vision_nodelet/ColorRecognize/Ix",Ki_x_,0);
        n_.param<double>("/whud_vision_nodelet/ColorRecognize/Dx",Kd_x_,0);
        n_.param<double>("/whud_vision_nodelet/ColorRecognize/Py",Kp_y_,1);
        n_.param<double>("/whud_vision_nodelet/ColorRecognize/Iy",Ki_y_,0);
        n_.param<double>("/whud_vision_nodelet/ColorRecognize/Dy",Kd_y_,0);   
        n_.param<bool>("/whud_vision_nodelet/ColorRecognize/flag_hist",flag_hist_,true);

        pid_x_.PID_init(0, 1, 0 ,0);
        pid_y_.PID_init(0, 1, 0 ,0);
        rate_divide_ = 4.0;
        HLow_ = 29; HHigh_ = 32; SLow_ = 43; SHigh_ = 255; VLow_ = 46; VHigh_ = 255;
        dynamic_func_ = boost::bind(&ColorRecognize::dynamic_callback,this,boost::placeholders::_1,boost::placeholders::_2);
        dynamic_server_.setCallback(dynamic_func_);
        ROS_INFO("ColorRecognize Init");                                               
    }

    void ColorRecognize::UpdateImage(Mat image)
    {
        image_now_ = image.clone();
        //ros::spinOnce();
    }
    
    void ColorRecognize::color_recognize()
    {
        Mat img_deal = image_now_.clone();
        Mat imgThresholded;
        Mat imgHSV;
        cvtColor(img_deal,imgHSV,COLOR_BGR2HSV);       
        vector<Mat> HSV_channels;

        if(flag_hist_)
        {
            split(imgHSV,HSV_channels);
            equalizeHist(HSV_channels[0],HSV_channels[0]);
            merge(HSV_channels,imgHSV);            
        }


        inRange(imgHSV, Scalar(HLow_, SLow_, VLow_), Scalar(HHigh_, SHigh_, VHigh_), imgThresholded);
        Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
		morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element);
		morphologyEx(imgThresholded, imgThresholded, MORPH_CLOSE, element);

        vector<vector<Point>> contours;
	    vector<Vec4i> hierarchy;
	    findContours(imgThresholded,contours,hierarchy,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE,Point());

        double max_area = 0;
        bool contour_flag = false;
        Rect rect;
        for(auto contour:contours)
        {
            contour_flag = true;
            double area = contourArea(contour);
            if(area > max_area)
            {
                max_area = area;
                rect = boundingRect(contour);
            }
        }
        if(contour_flag && max_area > lower_area_ && max_area < upper_area_)
        {
                double obj_center_x = rect.x + rect.width/2.0;
                double obj_center_y = rect.y + rect.height/2.0;

                rectangle(img_deal,rect,Scalar(0,255,0),2);

                double delta_x = obj_center_x - center_x_;
                double delta_y = obj_center_y - center_y_;

                pid_x_.PID_set(Kp_x_,Ki_x_, Kd_x_);
                pid_y_.PID_set(Kp_y_,Ki_y_, Kd_y_);
                
                speed_y_ = pid_x_.PID_increment(delta_x/center_x_/rate_divide_);
                speed_x_ = pid_y_.PID_increment(delta_y/center_y_/rate_divide_);

                speed_y_ = speed_y_ > max_speed_? max_speed_ : speed_y_;
                speed_y_ = speed_y_ < -max_speed_? -max_speed_ : speed_y_;
                speed_x_ = speed_x_ > max_speed_? max_speed_ : speed_x_;
                speed_x_ = speed_x_ < -max_speed_? -max_speed_ : speed_x_;
        }
        else
        {
            speed_y_ = 0;
            speed_x_ = 0;            
        }
        sensor_msgs::ImageConstPtr img_msg = cv_bridge::CvImage(std_msgs::Header(),"bgr8",img_deal).toImageMsg();
        img_pub_.publish(img_msg);           
    }

    void ColorRecognize::execute(const whud_vision::ImageProcessingGoalConstPtr& goal)
    {
        HLow_ = 29; HHigh_ = 32; SLow_ = 43; SHigh_ = 255; VLow_ = 46; VHigh_ = 255;
        ros::Rate rate(recognize_freq_);
        whud_vision::ImageProcessingFeedback feedback;
        ROS_INFO("ColorRecognize Action Server Execute");

        while(!server_.isPreemptRequested())
        {
            color_recognize();
            feedback.twist.linear.x = 0;
            feedback.twist.linear.y = speed_y_;
            server_.publishFeedback(feedback);
            rate.sleep();
        }
        int picture_index;
        file_.open("/home/nvidia/Code/simple_ws/src/whud_vision/config/picture_index.txt",ios::in);
        file_>>picture_index;
        file_.close();
        imwrite("/home/nvidia/Code/simple_ws/src/whud_vision/picture/BarCode"+to_string(picture_index)+".jpg", image_now_);
        picture_index++;
        file_.open("/home/nvidia/Code/simple_ws/src/whud_vision/config/picture_index.txt",ios::out);
        file_<<picture_index;
        file_.close();
        ROS_INFO("ColorRecognize Execute Finish");
        server_.setSucceeded();
	server_.start();
    }

    void ColorRecognize::dynamic_callback(whud_vision::whud_vision_dynamicConfig &config, uint32_t level)
    {
        ROS_INFO("Dynamic Params Changed");
        Kp_x_ = config.Kp1;
        Ki_x_ = config.Ki1;
        Kd_x_ = config.Kd1;

        Kp_y_ = config.Kp2;
        Ki_y_ = config.Ki2;
        Kd_y_ = config.Kd2;

        HLow_ = config.H_low;
        HHigh_ = config.H_high;
        SLow_ = config.S_low;
        SHigh_ = config.S_high;
        VLow_ = config.V_low;
        VHigh_ = config.V_high;

        upper_area_ = config.upper_area;
        lower_area_ = config.lower_area;
    }
}
PLUGINLIB_EXPORT_CLASS(vision_plugins::ColorRecognize, vision_base::VisionBase);
