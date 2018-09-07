
/* Read Quadrature Encoder
  * Connect Encoder to Pins encoder0PinA, encoder0PinB, and +5V.
  *
  * Sketch by max wolf / www.meso.net
  * v. 0.1 - very basic functions - mw 20061220
  * v. 0.2 - conversion to standard units, documentation
  */  

/* 
 * This driver is used to report and control a pillar element
 * with quadrature encoders.
 *
 * This is exposed through ROS via the {namespace}/pillar/current and 
 * {namespace}/pillar/request topics, which report the position in meters
 * and take a position request in meters, respetively.
 */
#include "string.h"
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/DiagnosticArray.h>

#include "DualMC33926MotorShield.h"

DualMC33926MotorShield md;
ros::NodeHandle  nh;

 diagnostic_msgs::DiagnosticStatus diagStatus;
 diagnostic_msgs::DiagnosticArray diagAr;

 float act_pos =0.0,des_pos=0.0,err = 40.0, gain= 0.0;
 int val;
 float pos[1];
 float effort[1] = {0.0};
 float vel[1] = {0.0};
 char *names[] = {"pillar_telescope_joint"};
 sensor_msgs::JointState cur_state; 
 int inputPinA = 10;
 int inputPinB = 9;
 int encoder0PinA = 5;
 int encoder0PinB = 6;
 float encoder0Pos = 0.0;
 int encoder0PinALast = LOW;
 int n = LOW,a =LOW,b=LOW;
 char junk = ' ';


 std_msgs::Float32 enc_data; 
 //Publish the encoder data
 ros::Publisher p("/pillar/joint_states", &cur_state);
 ros::Publisher diag_pub("/diagnostics", &diagAr);

 void update_state(){
  // convert from mm to meters
  pos[0] = act_pos / 1000;
  cur_state.position = pos;
  cur_state.header.stamp = nh.now();
//  diag_pub.publish(&diagAr);
  p.publish(&cur_state);
 }


//Get desired input position while generating the actual position of the pillar in mm
 void pillar_cb( const std_msgs::Float32& cmd_msg){
  
  //convert from meters to mm (which is expected by the controller)
  des_pos = cmd_msg.data * 1000;

  if (des_pos < 0.0){ //Homing condition
   md.setM1Speed(400);
   delay(5000);
  }
  else{
    while(abs(des_pos-act_pos)>0.75)
    {
     // Read Current Position   
     n = digitalRead(encoder0PinA);
     if ((encoder0PinALast == LOW) && (n == HIGH)) {
       if (digitalRead(encoder0PinB) == LOW) {
         encoder0Pos--;
       } else {
         encoder0Pos++;
       }
       act_pos = (encoder0Pos*9.0)/14.0;
       update_state();
     } 
     encoder0PinALast = n;
  
  
     //Feedback proportional gain control
     gain = -70.0;
     err = gain*(des_pos - act_pos);
  
  
    //Saturation
    if (err>400.0) err =400.0;
    if (err<-400.0) err = -400.0;
    //Stop chatter due to inaccuracies.
    if (abs(err)<10.0)  md.setM1Speed(0);
    else md.setM1Speed((int)err);
     
    if (md.getFault())
    {
      diagAr.header.stamp = nh.now();
      diagStatus.level = diagStatus.ERROR;
      diagAr.status[0] = diagStatus;
      diag_pub.publish(&diagAr);
      Serial.println("fault");
      while(true){
         diag_pub.publish(&diagAr);
         delay(10);
      }
    }
    
    }
  } 
}
 //Subscribe to the node that provides desired position
 ros::Subscriber<std_msgs::Float32> s("/pillar/command", &pillar_cb);

 void setup() { 
   pinMode (encoder0PinA,INPUT);
   pinMode (encoder0PinB,INPUT);
   pinMode (inputPinA,INPUT);
   pinMode (inputPinB,INPUT);
   //Serial.println("Initializing Pillar...");
   //Enforce Baud rate in ROS  
   nh.getHardware()->setBaud(115200);
   
   //Initialize joint states
   cur_state.name_length = 1;
   cur_state.velocity_length = 1;
   cur_state.position_length = 1; /// here used for arduino time
   cur_state.effort_length = 1;
   pos[0] = 1.0;
   cur_state.header.frame_id = "pillar_telescope_link";
   cur_state.name = names;
   cur_state.position = pos;
   cur_state.velocity = vel;
   cur_state.effort = effort;
   
   diagStatus.name = "pillar_driver";
   diagStatus.message = "Streaming";
   diagStatus.level = diagStatus.OK;
   diagAr.status_length = 1;
   diagAr.status[0] = diagStatus;

   //Setup the ROS nodes
   nh.initNode();
   nh.advertise(p);
   nh.advertise(diag_pub);
   nh.subscribe(s);
   //Initialize the motor controller
   md.init();
   
 } 

 void loop() {
  update_state();
  nh.spinOnce();
  delay(10);
  }
 
