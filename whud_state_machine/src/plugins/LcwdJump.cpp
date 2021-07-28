/**
 * @file 
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
class LcwdJump : public PluginBase {
public:

  LcwdJump() : PluginBase(){};

  void OnInit(MavRosPublisher &mavros_pub) {
    PluginBase::OnInit(mavros_pub);
  }

  bool SetTask(ros::V_string param) {
    PluginBase::SetTask(param);
    current_stage_ = atoi(param[0].c_str());
    rewrite_ = atoi(param[1].c_str());
    // if(rewrite_){
    //     file_.open("/home/nvidia/Code/simple_ws/src/whud_state_machine/config/task_stage.txt",ios::out);
    //     file_<<current_stage_;
    //     ROS_INFO("Write in %d",current_stage_);
    //     file_.close();
    // }
    return true;
  }


  virtual void TaskSpin() override {
       if(rewrite_){
        file_.open("/home/nvidia/Code/simple_ws/src/whud_state_machine/config/task_stage.txt",ios::out);
        file_<<current_stage_;
        ROS_INFO("Write in %d",current_stage_);
        file_.close();
      }     
        task_status_ = TaskStatus::DONE;
  }

  virtual void StopTask() override {}

private:
  ofstream file_;
  int current_stage_;
  bool rewrite_;
  FILE *fp=NULL;
};

}  // namespace whud_state_machine

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(whud_state_machine::LcwdJump,
                       whud_state_machine::PluginBase)
