#include <ros/ros.h>
#include <stdlib.h>
#include "DataStructure.hpp"
#include "StateMachinePlugin.hpp"
#include <std_msgs/Bool.h>


using namespace std;

namespace whud_state_machine {
class WhudDisarm : public PluginBase {
public:

  WhudDisarm() : PluginBase(){};

  void OnInit(MavRosPublisher &mavros_pub) {
    PluginBase::OnInit(mavros_pub);
    disarm_pub_ = nh_.advertise<std_msgs::Bool>("/mavros/whud_basic/disarm",1);
  }

  bool SetTask(ros::V_string param) {
    PluginBase::SetTask(param);
    return true;
  }


  virtual void TaskSpin() override {
      std_msgs::Bool disarm;
      disarm.data = true;
      disarm_pub_.publish(disarm);

    task_status_ = TaskStatus::DONE;
  }

  virtual void StopTask() override {}


private:
    ros::NodeHandle nh_;
    ros::Publisher disarm_pub_;

};

}  // namespace whud_state_machine

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(whud_state_machine::WhudDisarm,
                       whud_state_machine::PluginBase)
