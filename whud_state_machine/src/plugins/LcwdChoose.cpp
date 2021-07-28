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
#include <stdlib.h>
#include <fstream>
#include "DataStructure.hpp"
#include "StateMachinePlugin.hpp"


using namespace std;

namespace whud_state_machine {
class LcwdChoose : public PluginBase {
public:

  LcwdChoose() : PluginBase(){};

  void OnInit(MavRosPublisher &mavros_pub) {
    PluginBase::OnInit(mavros_pub);
    task_stage_ = 1;
  }

  bool SetTask(ros::V_string param) {
    PluginBase::SetTask(param);
    file_.open("/home/nvidia/Code/simple_ws/src/whud_state_machine/config/task_stage.txt",ios::in);
    if(file_.fail())
    {
      ROS_INFO("Read Error");
    }
    target_stage_ = atoi(param[0].c_str());
    ROS_INFO("Param: %d",target_stage_);
    file_>>task_stage_;
    ROS_INFO("File: %d",task_stage_);
    file_.close();
    return true;
  }


  virtual void TaskSpin() override {
      ROS_INFO("Param: %d  File: %d",target_stage_,task_stage_);
      if(task_stage_ == target_stage_)
        interrupt_signal_ = true;
      else
        task_status_ = TaskStatus::DONE;
  }

  virtual void StopTask() override {}

private:
  fstream file_;
  int task_stage_;
  int target_stage_;
};

}  // namespace whud_state_machine

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(whud_state_machine::LcwdChoose,
                       whud_state_machine::PluginBase)
