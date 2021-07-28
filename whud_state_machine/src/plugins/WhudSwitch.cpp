/**
 * @file WhudBasicControl.cpp
 * @author LauZanMo (LauZanMo@whu.edu.cn)
 * @author Chen Junpeng (chenjunpeng@whu.edu.cn)
 * @brief whud_basic_control plugin
 * @version 1.0
 * @date 2021-06-04
 *
 * @copyright Copyright (c) 2021 WHU-Drones
 *
 */
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <stdlib.h>

#include "DataStructure.hpp"
#include "StateMachinePlugin.hpp"


using namespace std;

namespace whud_state_machine {
class WhudSwitch : public PluginBase {
public:

  WhudSwitch() : PluginBase(),nh_(""){};

  void OnInit(MavRosPublisher &mavros_pub) {
    PluginBase::OnInit(mavros_pub);
    botton_state_sub_ = nh_.subscribe("/botton_state",1, &WhudSwitch::BottonStartCb,this);
    ok_pub_ = nh_.advertise<std_msgs::String>("/led_on_off",1);
    set_mode_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/mavros/whud_basic/set_mode",5);
  }

  bool SetTask(ros::V_string param) {
    PluginBase::SetTask(param);
    std_msgs::String msg;
    msg.data = "on";
    for(int i=0;i<2;i++)
    {
      ok_pub_.publish(msg);     
    }
    botton_state_ = false;
    if(base_nh_.hasParam(param[0])){
      switch_name_=param[0];
      base_nh_.setParam(switch_name_,false);
      return true;
    }
    else
      return false;    
  }


  virtual void TaskSpin() override {
      base_nh_.param<bool>(switch_name_,switch_state_,true);

      if(switch_state_ || botton_state_){
        std_msgs::String msg;
        msg.data = "off";
        for(int i=0;i<2;i++)
        {
          ok_pub_.publish(msg);     
        }
        set_mode_.data.clear();
        set_mode_.data.push_back(atof("0"));
        set_mode_.data.push_back(atof("35"));
        set_mode_pub_.publish(set_mode_);
        task_status_ = TaskStatus::DONE;
      }
  }

  virtual void StopTask() override {}

  void BottonStartCb(const std_msgs::String::ConstPtr &msg)
  {
    botton_state_ = true;
  }

private:
  ros::NodeHandle nh_;
  bool switch_state_;
  std::string switch_name_;
  bool botton_state_;
  ros::Subscriber botton_state_sub_;
  ros::Publisher ok_pub_;
  ros::Publisher set_mode_pub_;
  std_msgs::Float64MultiArray set_mode_;
};

}  // namespace whud_state_machine

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(whud_state_machine::WhudSwitch,
                       whud_state_machine::PluginBase)
