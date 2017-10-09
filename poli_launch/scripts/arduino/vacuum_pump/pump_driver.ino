#include "DualMC33926MotorShield.h"

DualMC33926MotorShield md;

bool flag1 = false;
bool flag2 = false;
int status = 0;

void stopIfFault()
{
  if (md.getFault())
  {
    Serial.println("fault");
    while(1);
  }
}

void letitgo()
{
  if(flag2){
    Serial.println("Releasing...");
    
    md.setM2Speed(400);
    delay(100);
    md.setM2Speed(0);
    flag2 = false;
 }
}

void grabitpls()
{
  if(flag1)
  {
    Serial.println("Grabber Engaged");
    md.setM1Speed(400);
  }
  else
  {
    Serial.println("Grabber Disengaged");
    md.setM1Speed(0);
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
  
   while (Serial.available() > 0) {

    // look for the next valid integer in the incoming serial stream:
    status = Serial.parseInt();
   

    if (status ==1)
    {
      flag1 = true;
      flag2 = false;
    }
    else if ((status ==2))
    {
      flag2=true;  
    }
    else
    {
      flag1 = false;
      flag2 = false;
    }

    grabitpls();
    letitgo();
   }    
   } 
  


