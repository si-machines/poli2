/*
 * LED ROS controller for the Eye + Ear LEDs for the PoliV2 project. 
 * This file also handles the limit switch for the pan/tilt motor system.
 *
 * The LED libraries that can be used generally are APA102 and Adafruit_DotStar.
 * This file MUST be flashed to an Arduino Mega or else non-deterministic behavior will ensue.
 * 
 * The DOTSTAR, matrix LEDs (eyes) use hardware SPI to streamline. This requires that the 
 * clock pin (yellow) go to 52 (SCK) and the green pin go to 51 (MOSI).
 * 
 * Author Maxwell Svetlik, Prashant Rao 
 */

#include <APA102.h>

#include <ros.h>
#include <std_msgs/Int16.h>
#include <poli_msgs/LedEye.h>
#include <poli_msgs/LedEar.h>
#include <rosserial_arduino/Test.h>

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_DotStarMatrix.h>
#include <Adafruit_DotStar.h>
#ifndef PSTR
 #define PSTR // Make Arduhino Due happy
#endif

#define DATAPIN  11
#define CLOCKPIN 13

Adafruit_DotStarMatrix matrix = Adafruit_DotStarMatrix(
  (uint8_t)8, (uint8_t)8, 2, 1, 
  DS_TILE_TOP   + DS_TILE_RIGHT   + DS_TILE_ROWS   + DS_TILE_PROGRESSIVE +
  DS_MATRIX_TOP + DS_MATRIX_RIGHT + DS_MATRIX_ROWS + DS_MATRIX_ZIGZAG,
  DOTSTAR_BRG);

ros::NodeHandle  nh;
using rosserial_arduino::Test;

// Define which pins to use.
const uint8_t ears_dataPin = 9;
const uint8_t ears_clockPin = 10;
const uint8_t limitSwitchPin = 4;

// Create an object for writing to the LED strip.
APA102<ears_dataPin, ears_clockPin> ledStrip;

const uint16_t colors[] = {
  matrix.Color(255, 0, 0), matrix.Color(0, 255, 0), matrix.Color(0, 0, 255), matrix.Color(202, 161, 241), matrix.Color(105, 203, 155)};
const uint16_t mycolor = colors[4];

// Set the number of LEDs to control.
const uint16_t ledCount = 90;

// Create a buffer for holding the colors (3 bytes per color).
rgb_color ear_colors[ledCount];

// Set the brightness to use (the maximum is 31).
const uint8_t max_brightness = 15;
uint8_t brightness = 10;

//Control variables
const uint8_t EAR_SOLID = 2;
const uint8_t EAR_GRADIENT = 3;
const uint8_t EAR_BREATH= 4;

const uint8_t EYE_NORMAL= 2;
const uint8_t EYE_BLINKY1= 3;
const uint8_t EYE_BLINKY2= 4;
const uint8_t EYE_WINKY= 5;
const uint8_t EYE_CRY= 6;
const uint8_t EYE_BLINKING_ACTION= 7;


int ear_mode = EAR_SOLID;
rgb_color ear_color = rgb_color(255, 255, 255);
boolean ear_enabled = true;

boolean eye_enabled = true;
int eye_mode = EYE_NORMAL;

/*
 * EYE methods
 */
void normal_eyes()
{
  matrix.fillCircle(3,3,2,mycolor);
  matrix.fillCircle(12,3,2,mycolor);
}

void blinky_eyes_1()
{
  matrix.drawLine(3,1,5,3,mycolor);
  matrix.drawLine(3,5,5,3,mycolor);
  matrix.drawLine(10,3,12,1,mycolor);
  matrix.drawLine(10,3,12,5,mycolor); 
}

void blinky_eyes_2()
{
  matrix.drawLine(1,3,5,3,mycolor);
  matrix.drawLine(10,3,14,3,mycolor); 
}

void winky_eyes()
{
  matrix.drawLine(3,1,5,3,mycolor);
  matrix.drawLine(3,5,5,3,mycolor);
  matrix.fillCircle(12,3,2,mycolor);
}

void cry_eyes()
{
  matrix.drawLine(1,3,5,3,mycolor);
  matrix.drawLine(3,3,3,6,mycolor);
  matrix.drawLine(10,3,14,3,mycolor);
  matrix.drawLine(12,3,12,6,mycolor); 
}

//blinks every 3 seconds
// DOESNT WORK. TOTDO
uint8_t startTime = millis();
uint8_t blinkTime = millis();
uint8_t lastTime = millis(); 

void blink_action()
{

  if(millis() - lastTime >= 3000 && millis() - lastTime <= 3500) {
    //start blinking
    blinky_eyes_2();
    return;
  }
  else if(millis() - lastTime > 3500){
    lastTime = millis();
    normal_eyes();
  }
 
}


void ear_srv_callback(const poli_msgs::LedEar::Request & req, poli_msgs::LedEar::Response & res){
  if(req.command == req.DISABLE){
      ear_enabled = false;
  }
  else if(req.command == req.ENABLE){
      ear_enabled = true;
  }
  else if(req.command == req.SOLID){
     switch(req.color){
       case 0:
          ear_color = rgb_color(255, 0, 0); break;
       case 1:
          ear_color = rgb_color(0, 0, 255); break;
       case 2:
          ear_color = rgb_color(0, 255, 0); break;
       case 3:
          ear_color = rgb_color(255, 255, 0); break;
       case 4:
          ear_color = rgb_color(255, 255, 255); break;
       default:
          ear_color = rgb_color(255, 255, 255); break;
     }
     ear_mode = EAR_SOLID;
     //TODO set color globally via an array
    
  } 
  else if(req.command == req.GRADIENT){
    ear_mode = EAR_GRADIENT;
  }
  res.response = req.SUCCESS;
  
}

void eye_srv_callback(const poli_msgs::LedEye::Request & req, poli_msgs::LedEye::Response & res){
  
  if(req.command == req.DISABLE){
      eye_enabled = false;
  }
  else if(req.command == req.ENABLE){
      eye_enabled = true;
  }
  else
      eye_mode = req.command;

  res.response = req.SUCCESS;
  
}

std_msgs::Int16 limit_switch_msg;
ros::ServiceServer<poli_msgs::LedEar::Request, poli_msgs::LedEar::Response> ear_server("led_ear",&ear_srv_callback);
ros::ServiceServer<poli_msgs::LedEye::Request, poli_msgs::LedEye::Response> eye_server("led_eye",&eye_srv_callback);
ros::Publisher limit_pub("pillar/limit_switch", &limit_switch_msg);

//TODO once the ears are further established, need to make the limits (i=40) more concrete
// and document it
uint16_t additor = 1;
void writeEar(){
  uint8_t time = millis() >> 2;
  if(ear_mode == EAR_SOLID){
    brightness = 15;
    for(uint16_t i = 40; i < ledCount; i++)
    {
      ear_colors[i] = ear_color;
    }
  }
  else if(ear_mode == EAR_GRADIENT){ 
    brightness = 15;
    for(uint16_t i = 40; i < ledCount; i++)
    {
      uint8_t x = time - i * 8;
      ear_colors[i] = rgb_color(255 - x, 255 - x, 255 - x);
    }
  }
  // THIS DOESNT WORK. TODO
  else if(ear_mode == EAR_BREATH){
    if(brightness < 1)
      additor = 1;
    else if(brightness > 15)
      additor = -1;
    brightness += additor;
     //for(uint8_t i = brightness; brightness > 0; i--){
     //   ledStrip.write(ear_colors, ledCount, i);
     //   delay(5);
     //}
     //for(uint8_t i = 0; i < brightness; i++){
     //   ledStrip.write(ear_colors, ledCount, i);
     //   delay(5);
     //}
  }
  if(ear_enabled)
     ledStrip.write(ear_colors, ledCount, brightness);
  else
     ledStrip.write(ear_colors, ledCount, 0);

}


void writeEye(){
  matrix.fillScreen(0);
  matrix.setCursor(2, 1);

  if(!eye_enabled){
      matrix.fillScreen(0);
      matrix.show();
  }
  else if(eye_mode == EYE_NORMAL){
      normal_eyes();
      matrix.show();
  }
  else if(eye_mode == EYE_BLINKY1){
      blinky_eyes_1();
  }
  else if(eye_mode== EYE_BLINKY2){
      blinky_eyes_2();
  }
  else if(eye_mode == EYE_WINKY){
      winky_eyes();
  }
  else if(eye_mode == EYE_CRY){
      cry_eyes();
  }
  else if(eye_mode == EYE_BLINKING_ACTION){
      blink_action();
  }
  
  if(eye_enabled){
    matrix.show();
  }
}  


int x    = 2;
int pass = 0;
int seed = 0;
//This method randomly blinks in different ways
void prashants_eye_loop(){
   seed = random(1,4);
    
    for (int i = 0;i<40;i++)
    { matrix.fillScreen(0);
      matrix.setCursor(x, 1);
      if(i<33)
      {
      normal_eyes();
      }
      else
      {
        switch(seed)
        {
          case 1:
          blinky_eyes_1();
          break;
          case 2:
          blinky_eyes_2();
          break;
          case 3:
          winky_eyes();
          break;
          case 4:
          cry_eyes();
                 
        }
        
      }
      matrix.show();
      delay(100);
 
    } 
}

void checkLimitSwitch(){
  if (digitalRead(limitSwitchPin) == HIGH) {
    limit_switch_msg.data = 1;
  }
  else{
    limit_switch_msg.data = 0; 
  }
  limit_pub.publish(&limit_switch_msg);
}

void setup()
{
  pinMode(limitSwitchPin, INPUT_PULLUP);
  matrix.begin();
  matrix.setTextWrap(false);
  matrix.setBrightness(10);
  
  nh.initNode();


  nh.advertiseService(ear_server);

  nh.advertiseService(eye_server);
  nh.advertise(limit_pub);
}

void loop()
{ 
  //prashants_eye_loop();
  writeEye();
  writeEar();
  checkLimitSwitch();
  
  nh.spinOnce();
  delay(10);
}
