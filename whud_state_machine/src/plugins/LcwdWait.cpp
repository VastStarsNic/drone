/**
 * @file 
 * @author 
 * @author 
 * @brief 
 * @version 1.0
 * @date 2021-06-04
 *
 * @copyright Copyright (c) 2021 WHU-Drones
 *
 */
#include <ros/ros.h>
#include <stdlib.h>
#include "DataStructure.hpp"
#include "StateMachinePlugin.hpp"
#include <std_msgs/String.h>
#include <unistd.h>

using namespace std;

namespace whud_state_machine {
class LcwdWait : public PluginBase {
public:

  LcwdWait() : PluginBase(),nh_(""){};

  void OnInit(MavRosPublisher &mavros_pub) {
    PluginBase::OnInit(mavros_pub);
    led_ctrl_pub_ = nh_.advertise<std_msgs::String>("/led_on_off",1);
  }

  bool SetTask(ros::V_string param) {
    PluginBase::SetTask(param);
    std_msgs::String msg;
    msg.data = "on";
    led_ctrl_pub_.publish(msg);
    sleep(2);
    msg.data = "off";
    led_ctrl_pub_.publish(msg);
    SetFinishDelay(atof(param[0].c_str()));

    return true;
  }


  virtual void TaskSpin() override {   
    task_status_ = TaskStatus::DONE;
  }

  virtual void StopTask() override {}

private:
  ros::Publisher led_ctrl_pub_;
  ros::NodeHandle nh_;
};

}  // namespace whud_state_machine

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(whud_state_machine::LcwdWait,
                       whud_state_machine::PluginBase);
