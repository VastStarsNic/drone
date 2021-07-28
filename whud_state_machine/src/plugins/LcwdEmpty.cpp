#include <ros/ros.h>
#include <stdlib.h>
#include "DataStructure.hpp"
#include "StateMachinePlugin.hpp"


using namespace std;

namespace whud_state_machine {
class LcwdEmpty : public PluginBase {
public:

  LcwdEmpty() : PluginBase(){};

  void OnInit(MavRosPublisher &mavros_pub) {
    PluginBase::OnInit(mavros_pub);
  }

  bool SetTask(ros::V_string param) {
    PluginBase::SetTask(param);
    return true;
  }


  virtual void TaskSpin() override {
        interrupt_signal_ = true;
  }

  virtual void StopTask() override {}

};

}  // namespace whud_state_machine

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(whud_state_machine::LcwdEmpty,
                       whud_state_machine::PluginBase)
