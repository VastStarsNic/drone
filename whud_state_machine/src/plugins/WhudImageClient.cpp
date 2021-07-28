/**
 * @file WhudImageClient.cpp
 * @author Liuzhihao (liuzhihao@whu.edu.cn)
 * @brief WhudImageClient plugin
 * @version 1.0
 * @date 2021-06-22
 * 
 * @copyright Copyright (c) 2021 WHU-Drones
 * 
 */
#include <ros/ros.h>
#include <whud_vision/ImageProcessingAction.h>
#include <stdlib.h>
#include <unistd.h>
#include <cmath>
#include "DataStructure.hpp"
#include "StateMachinePlugin.hpp"

#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

using namespace std;

namespace whud_state_machine {
typedef actionlib::SimpleActionClient<whud_vision::ImageProcessingAction> Client;
class WhudImageClient : public PluginBase {
public:
  /**
   * @brief Construct a new Whud Image Control object
   *
   * @note In this function, private node handle and will be initialized
   */
  WhudImageClient() : PluginBase(), nh_("~whud_image_client"),  image_client_("color_recognize",true){}

  /**
   * @brief Destroy the Whud Image Client object
   */
  ~WhudImageClient() {}

  virtual void OnInit(MavRosPublisher &mavros_pub) override {
    PluginBase::OnInit(mavros_pub);
    led_ctrl_pub_ = nh_.advertise<std_msgs::String>("/led_on_off",1);
  }

  /**
   * @brief Set the Image Task object
   * 
   * @param param A parameter list of size 2 needed by WhudImageClient plugin, where
   * param[0]: The pid judging time. If our robot keep still for time 
   * longer than this parameter, then our task is done.
   * param[1]: The delay time after task is done.
   * @retval true  Task is correctly set.
   * @retval false Task is incorrectly set.
   */
  virtual bool SetTask(ros::V_string param) override {
        PluginBase::SetTask(param);
        stop_judge_ = atof(param[0].c_str());
        SetFinishDelay(atof(param[1].c_str()));
        time_begin_ = timer_.current_expected.now().toSec();
        ROS_INFO(
          "Wait for image processing server and transform set up, please be sure they "
          "will be set up.");
        image_client_.waitForServer();

        ROS_INFO("Server Start");
        whud_vision::ImageProcessingGoal goal;
        goal.start = true;
        image_client_.sendGoal(
            goal,
            std::bind(&WhudImageClient::DoneCb, this, std::placeholders::_1,
                    std::placeholders::_2),
            std::bind(&WhudImageClient::ActiveCb, this),
            std::bind(&WhudImageClient::FeedbackCb, this, std::placeholders::_1));
    return true;
  }

  /**
   * @brief Judge if task is done.
   * 
   * @note If our robot keep speed of x any y axis smaller than our set value(0.03 here)
   * for time longer than param[0], then our task is done.
   * 
   */
  virtual void TaskSpin() override {
      mavros_pub_->cmd_vel_pub.publish(vel);
      if(fabs(vel.linear.x) < 0.07 && fabs(vel.linear.y) < 0.1 && !(vel.linear.x==0 && vel.linear.y==0)){
        if(timer_.current_expected.now().toSec()-time_begin_>=stop_judge_)
        {
          std_msgs::String msg;
          for(int i = 0; i<3;i++)
          {
            msg.data = "on";
            led_ctrl_pub_.publish(msg);
            usleep(50000);
            msg.data = "off";      
            led_ctrl_pub_.publish(msg);     
            usleep(50000); 
          }
          image_client_.cancelGoal();
        }
      }
      else{
          time_begin_=timer_.current_expected.now().toSec();
      }
  }

  /**
   * @brief Stop the action task
   * 
   */
  virtual void StopTask() override {
std_msgs::String msg;
          for(int i = 0; i<3;i++)
          {
            msg.data = "on";
            led_ctrl_pub_.publish(msg);
            usleep(50000);
            msg.data = "off";      
            led_ctrl_pub_.publish(msg);     
            usleep(50000); 
          }
    image_client_.cancelGoal();
    std_msgs::Bool conversion;
    conversion.data = true;
    mavros_pub_->conversion_pub.publish(conversion);
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher led_ctrl_pub_;
  geometry_msgs::Twist vel;
  ros::TimerEvent timer_;
  double time_begin_;
  double stop_judge_;
  Client image_client_;

  void ActiveCb() {}
  /**
   * @brief Use feedback from server to update control speed.
   * 
   * @param feedback A const pointer of size 2, where it first element means the 
   * speed of x axis and the second element means the speed of y axis.
   */
  void FeedbackCb(const whud_vision::ImageProcessingFeedbackConstPtr &feedback) {
      vel = feedback->twist;
  }

  /**
   * @brief Judge if our task is successfully done.
   * 
   * @param state Denote the state of action.
   * @param result The result of action.
   */
  void DoneCb(const actionlib::SimpleClientGoalState &state,
              const whud_vision::ImageProcessingResultConstPtr &result) {
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
      task_status_ = TaskStatus::DONE;
      std_msgs::Bool conversion;
      conversion.data = true;
      mavros_pub_->conversion_pub.publish(conversion);
    }
  }

};

}  // namespace whud_state_machine

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(whud_state_machine::WhudImageClient,
                       whud_state_machine::PluginBase)
