/* Read Quadrature Encoder
  * Connect Encoder to Pins encoder0PinA, encoder0PinB, and +5V.
  *
  * Sketch by max wolf / www.meso.net
  * v. 0.1 - very basic functions - mw 20061220
  *
  */  

#include <ros.h>
#include <std_msgs/Float32.h>

#include "DualMC33926MotorShield.h"

DualMC33926MotorShield md;
ros::NodeHandle  nh;

void stopIfFault()
{
  if (md.getFault())
  {
    Serial.println("fault");
    while(1);
  }
}
 float act_pos =0.0,des_pos=0.0,err = 40.0, gain= 0.0;
 int val; 
 int inputPinA = 10;
 int inputPinB = 9;
 int encoder0PinA = 5;
 int encoder0PinB = 6;
 float encoder0Pos = 0.0;
 int encoder0PinALast = LOW;
 int n = LOW,a =LOW,b=LOW;
 char junk = ' ';

//Get desired input position while generating the actual position of the pillar in mm
 void pillar_cb( const std_msgs::Float32& cmd_msg){
  
  des_pos = cmd_msg.data;
  if (des_pos == -1.0){ //Homing condition
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
     //Serial.print (act_pos);
     //Serial.println (" mm");
     
   } 
   encoder0PinALast = n;


   //Feedback proportional gain control
   gain = -70.0;
   err = gain*(des_pos - act_pos);


  //Saturation
  if (err>400.0) err =400.0;
  if (err<-400.0) err = -400.0;
  //Serial.print("Motor Speed = ");
  //Serial.println(err);
  //Stop chatter due to inaccuracies.
  if (abs(err)<10.0)  md.setM1Speed(0);
  else md.setM1Speed((int)err);
   
  stopIfFault();
  
  }
  } 
}

 std_msgs::Float32 enc_data;
 //Subscribe to the node that provides desired position
 ros::Subscriber<std_msgs::Float32> s("Desired", &pillar_cb);
 //Publish the encoder data
 ros::Publisher p("Current", &enc_data);
 
 void setup() { 
   pinMode (encoder0PinA,INPUT);
   pinMode (encoder0PinB,INPUT);
   pinMode (inputPinA,INPUT);
   pinMode (inputPinB,INPUT);
   //Serial.begin (115200);
   //Serial.println("Initializing Pillar...");
   //Enforce Baud rate in ROS  
   nh.getHardware()->setBaud(115200);
   //Setup the ROS nodes
   nh.initNode();
   nh.advertise(p);
   nh.subscribe(s);
   //Initialize the motor controller
   md.init();
   //Go to home position for 3 secs 
   
 } 

 void loop() { 
  enc_data.data = act_pos;
  p.publish(&enc_data);
  nh.spinOnce();
  delay(10);
  }
 
