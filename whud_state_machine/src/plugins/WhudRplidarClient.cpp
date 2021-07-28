#include <ros/ros.h>
#include <stdlib.h>
#include "DataStructure.hpp"
#include "StateMachinePlugin.hpp"
#include "lcwd_rplidar_measure/RplidarMeasureAction.h"
#include "whud_vision/ImageProcessingAction.h"
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Bool.h>

using namespace std;

namespace whud_state_machine {
typedef actionlib::SimpleActionClient<lcwd_rplidar_measure::RplidarMeasureAction> RplidarClient;
class WhudRplidarClient : public PluginBase {
public:

  WhudRplidarClient() : PluginBase(), nh_("~whud_rplidar_client"), rplidar_client_("rplidar_pass_frame",true){};

  void OnInit(MavRosPublisher &mavros_pub) {
    PluginBase::OnInit(mavros_pub);
  }

  bool SetTask(ros::V_string param) {
    PluginBase::SetTask(param);
	
    ROS_INFO(
        "Wait for rplidar measure server, please be sure they "
        "will be set up.");
    rplidar_client_.waitForServer();
    ROS_INFO("Rplidar Measure Server Start");    
	  SetFinishDelay(atof(param[0].c_str()));

    lcwd_rplidar_measure::RplidarMeasureGoal goal;
    goal.start = true;
    rplidar_client_.sendGoal(
        goal,
        std::bind(&WhudRplidarClient::DoneCb, this, std::placeholders::_1,
                std::placeholders::_2),
        std::bind(&WhudRplidarClient::ActiveCb, this),
        std::bind(&WhudRplidarClient::FeedbackCb, this, std::placeholders::_1));
    return true;
  }


  virtual void TaskSpin() override {}

  virtual void StopTask() override {
    rplidar_client_.cancelGoal();
    std_msgs::Bool conversion;
    conversion.data = true;
    mavros_pub_->conversion_pub.publish(conversion);
  }

private:
    RplidarClient rplidar_client_;
    ros::NodeHandle nh_;

  void ActiveCb() {}
  /**
   * @brief Use feedback from server to update control speed.
   * 
   * @param feedback A const pointer of size 2, where it first element means the 
   * speed of x axis and the second element means the speed of y axis.
   */
  void FeedbackCb(const lcwd_rplidar_measure::RplidarMeasureFeedbackConstPtr &feedback) {
      mavros_pub_->cmd_vel_pub.publish(feedback->twist);
      ROS_INFO("x_speed: %f, y_speed: %f",feedback->twist.linear.x,feedback->twist.linear.y);
  }

  /**
   * @brief Judge if our task is successfully done.
   * 
   * @param state Denote the state of action.
   * @param result The result of action.
   */
  void DoneCb(const actionlib::SimpleClientGoalState &state,
              const lcwd_rplidar_measure::RplidarMeasureResultConstPtr &result) {
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
PLUGINLIB_EXPORT_CLASS(whud_state_machine::WhudRplidarClient,
                       whud_state_machine::PluginBase)
