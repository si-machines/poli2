

/*
 * Driver that controls the de/activation of an electric pump
 * Used with a DualMC33926 Motor controller.
 *
 */


#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Int16.h>
#include <poli_msgs/GripperPump.h>

#include "DualMC33926MotorShield.h"

DualMC33926MotorShield md;

int status = 0;


  ros::NodeHandle nh_;

  void executeCB(const poli_msgs::GripperPump::Request & req, poli_msgs::GripperPump::Response & res)
  {
    // helper variables
    bool success = true;

    // publish info to the console for the user
    nh_.loginfo("Executing gripper pump action...");

    // start executing the action
    if(req.command == req.ACTIVATE){
      md.setM1Speed(400);
    }
    else if(req.command == req.DEACTIVATE){
      md.setM1Speed(0);
    }
    else if(req.command == req.RELEASE){
      md.setM2Speed(400);
      delay(100);
      md.setM2Speed(0);
    }


    //md.GetFault();
    res.success = req.SUCCESS;
    nh_.loginfo("Succeeded");
  }

void stopIfFault()
{
  if (md.getFault())
  {
    nh_.logerror("Gripper pump driver faulted.");
    nh_.logerror("Shutting down pump driver.");
    Serial.println("fault");
    while(1);
  }
}

ros::ServiceServer<poli_msgs::GripperPump::Request, poli_msgs::GripperPump::Response> server("poli/gripper_vacuum", &executeCB);

void setup() {
  //Enforce Baud rate in ROS  
  nh_.getHardware()->setBaud(115200);
  //Serial.begin(115200);
  Serial.println("Ready to receive commands...");
  //initialize the motor controller
  md.init();
  md.setM1Speed(0);
  md.setM2Speed(0);
  nh_.initNode();

  nh_.advertiseService(server);
}

void loop() {
   nh_.spinOnce();
}
