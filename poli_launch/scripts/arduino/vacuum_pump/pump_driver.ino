
/*
 * Driver that controls the de/activation of an electric pump
 * Used with a DualMC33926 Motor controller.
 *
 */


#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Int16.h>
#include <actionlib/server/simple_action_server.h>
#include <poli_msgs/GripperPumpAction.h>

#include "DualMC33926MotorShield.h"

DualMC33926MotorShield md;

bool flag1 = false;
bool flag2 = false;
int status = 0;

class GripperPumpAction
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<poli_msgs::GripperPumpAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  poli_msgs::GripperPumpFeedback feedback_;
  poli_msgs::GripperPumpResult result_;
  poli_msgs::GripperPumpAction gpa;

public:

  GripperPumpAction(std::string name) :
    as_(nh_, name, boost::bind(&GripperPumpAction::executeCB, this, _1), false),
    action_name_(name)
  {
    nh.initNode();
    as_.start();
  }

  void executeCB(const poli_msgs::GripperPumpGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(1);
    bool success = true;

    // publish info to the console for the user
    ROS_INFO("Executing gripper pump action...");

    // start executing the action
    if(goal->command == gpa.ACTIVATE){
      md.setM1Speed(400);
    }
    else if(goal->command == gpa.DEACTIVATE){
      md.setM1Speed(0);
    }
    else if(goal->command == gpa.RELEASE){
      md.setM2Speed(400);
      delay(100);
      md.setM2Speed(0);
      flag2 = false;
    }


    //md.GetFault();
    result.success = gpa.SUCCESS;
    ROS_INFO("%s: Succeeded", action_name_.c_str());
    // set the action state to succeeded
    as_.setSucceeded(result_);

  }
};



void stopIfFault()
{
  if (md.getFault())
  {
    ROS_ERROR("Gripper pump driver faulted.");
    ROS_ERROR("Shutting down pump driver.");
    Serial.println("fault");
    while(1);
  }
}

void setup() {

  Serial.begin(115200);
  Serial.println("Ready to receive commands...");
  //initialize the motor controller
  md.init();
  md.setM1Speed(0);
  md.setM2Speed(0);
}

void loop() {
   GripperPumpAction gripperPump("gripperpump");
   ros::spin();

