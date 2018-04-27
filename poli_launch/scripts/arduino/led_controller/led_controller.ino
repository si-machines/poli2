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
const uint16_t eye_color = colors[4];
const uint16_t mouth_color = colors[4];

//Delete this once refactor complete
const uint16_t mycolor = colors[4];

// Set the number of LEDs to control.
const uint16_t ledCount = 90;

// Create a buffer for holding the colors (3 bytes per color).
rgb_color ear_colors[ledCount];

// Set the brightness to use (the maximum is 31).
const uint8_t max_brightness = 10;
uint8_t brightness = 5;

//Control variables
const uint8_t EAR_SOLID = 2;
const uint8_t EAR_GRADIENT = 3;
const uint8_t EAR_BREATH= 4;

//Eye shape constants
const uint8_t NORMAL = 0;
const uint8_t CLOSE = 1;
const uint8_t CRY = 2;
const uint8_t SQUINT = 3;
const uint8_t WINK = 4;
const uint8_t MAD = 5;
const uint8_t SIDELEFT = 6;
const uint8_t HAPPY = 7;
const uint8_t SAD = 8;
const uint8_t SIDERIGHT = 9;
const uint8_t HEART = 10;
const uint8_t SLEEPY = 11;
const uint8_t SLEEPY2 = 12;

//Mouth shape constants
const uint8_t FLAT = 0;
const uint8_t GRIN = 1;
const uint8_t GRIMACE = 2;
const uint8_t OPEN = 3;
const uint8_t LONGFACE = 4;
const uint8_t SMILE = 5;
const uint8_t FROWN = 6;
const uint8_t BIGOPEN = 7;
const uint8_t SQUIGGLE = 8;

//Full face constants
const uint8_t HI = 0;


int ear_mode = EAR_BREATH;
rgb_color ear_color = rgb_color(255, 255, 255);
boolean ear_enabled = true;

boolean eye_enabled = true;
boolean face_mode = false;
int eye_shape = 0;
int eye_direction = 0;
int mouth_shape = 0;
int mouth_direction = 0;
int face_shape = 0;

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

void draw_eyes()
{
  switch(eye_shape){
  case NORMAL:
    matrix.fillRect(2,2,3,3,eye_color);
    matrix.drawPixel(3,1,eye_color);
    matrix.drawPixel(3,5,eye_color);
    matrix.fillRect(11,2,3,3,eye_color);
    matrix.drawPixel(12,1,eye_color);
    matrix.drawPixel(12,5,eye_color);
    break;
  case CLOSE:
    break;
  case CRY:
    break;
  case SQUINT:
    break;
  case WINK:
    break;
  case MAD:
    break;
  case SIDELEFT:
    break;
  case SIDERIGHT:
    break;
  case HEART:
    break;
  case SLEEPY:
    break;
  case SLEEPY2:
    break;
  default:
    matrix.fillRect(2,2,3,3,eye_color);
    matrix.drawPixel(3,1,eye_color);
    matrix.drawPixel(3,5,eye_color);
    matrix.fillRect(11,2,3,3,eye_color);
    matrix.drawPixel(12,1,eye_color);
    matrix.drawPixel(12,5,eye_color);
    break;
  } 
}

void draw_mouth()
{
  switch(eye_shape){
  case FLAT:
    matrix.drawLine(6,6,9,6,mouth_color);
    break;
  case GRIN:
    break;
  case GRIMACE:
    break;
  case OPEN:
    break;
  case LONGFACE:
    break;
  case SMILE:
    break;
  case FROWN:  
    break;
  case BIGOPEN:
    break;
  case SQUIGGLE:
    break;
  default:
    matrix.drawLine(6,6,9,6,mouth_color);
    break;
  } 
}

void draw_face()
{
  switch(face_shape){
  case HI:
    break;
  default:
    break;
  }
}


void writeEye(){
  matrix.fillScreen(0);
  matrix.setCursor(2, 1);

  if(!eye_enabled){
    matrix.fillScreen(0);
    matrix.show();
  }

  if(face_mode){
    draw_face();
  }
  else{
    draw_eyes();
    draw_mouth();
  }

  if(eye_enabled){
    matrix.show();
  }
}  


/*
 * blink_action blinks normally in a deterministic fashion. 
 * Blinking is done in 'rounds' or a percentage of calls to this function due to the 
 * unparallelized nature of the arduino. 
 * Using delays pauses other LED actions on the ears, such as pulsing or gradients. 
 * Thus using this scheme of cycles is necessary for animations.
 * 
 * You can change blinking rates by the following parameters:
 *    round_cutoff = the number of cycles that the eyes will be 'normal' or 'open'
 *    blink_difference = the number of cycles the eyes will be 'blinking' or 'closed'
 */
int round_cutoff = 60;
int blink_difference = 5;
int blink_round = 0;
void blink_action()
{
  matrix.fillScreen(0);
  matrix.setCursor(0, 1);
  if(blink_round < round_cutoff){
    normal_eyes();
  }
  else
  {
    blinky_eyes_2();
  }
  blink_round += 1;
  if(blink_round > round_cutoff + blink_difference)
    blink_round = 0;
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
      ear_color = rgb_color(255, 0, 0); 
      break;
    case 1:
      ear_color = rgb_color(0, 0, 255); 
      break;
    case 2:
      ear_color = rgb_color(0, 255, 0); 
      break;
    case 3:
      ear_color = rgb_color(255, 255, 0); 
      break;
    case 4:
      ear_color = rgb_color(255, 255, 255); 
      break;
    default:
      ear_color = rgb_color(255, 255, 255); 
      break;
    }
    ear_mode = EAR_SOLID;

  } 
  else if(req.command == req.GRADIENT){
    ear_mode = EAR_GRADIENT;
  }
  else if(req.command == req.BREATH){
    ear_mode = EAR_BREATH; 
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
  else{
    if(req.update_what == req.FULL_FACE){
      face_mode = true;
      face_shape = req.face_shape;
    }
    else if(req.update_what == req.EYES || req.update_what == req.BOTH){
      face_mode = false;
      if(req.update_what_eyes == req.DIRECTION || req.update_what_eyes == req.BOTH){
        eye_direction = req.eye_direction;
      } 
      else if(req.update_what_eyes == req.SHAPE || req.update_what_eyes == req.BOTH){
        eye_shape = req.eye_shape;
      }
      else if(req.update_what == req.MOUTH || req.update_what == req.BOTH){
        face_mode = false;
        mouth_shape = req.mouth_shape;
      }
    }
    res.response = req.SUCCESS;
  }
}

std_msgs::Int16 limit_switch_msg;
ros::ServiceServer<poli_msgs::LedEar::Request, poli_msgs::LedEar::Response> ear_server("led_ear",&ear_srv_callback);
ros::ServiceServer<poli_msgs::LedEye::Request, poli_msgs::LedEye::Response> eye_server("led_eye",&eye_srv_callback);
ros::Publisher limit_pub("pillar/limit_switch", &limit_switch_msg);

void initialize_ear_colors(){
  for(uint16_t i = 50; i < ledCount; i++)
  {
    ear_colors[i] = rgb_color(255, 255, 255);
  }
}

//TODO once the both ears are established, need to make the limits (i=50) more concrete
// and document it
uint16_t additor = 1;
void writeEar(){
  uint8_t time = millis() >> 2;
  if(ear_mode == EAR_SOLID){
    brightness = 10;
    for(uint16_t i = 50; i < ledCount; i++)
    {
      ear_colors[i] = ear_color;
    }
  }
  else if(ear_mode == EAR_GRADIENT){ 
    brightness = 10;
    for(uint16_t i = 50; i < ledCount; i++)
    {
      uint8_t x = time - i * 8;
      ear_colors[i] = rgb_color(255 - x, 255 - x, 255 - x);
    }
  }
  else if(ear_mode == EAR_BREATH){
    if(brightness <= 7)
      additor = 1;
    else if(brightness >= 17)
      additor = -1;
    brightness += additor;
  }
  if(ear_enabled)
    ledStrip.write(ear_colors, ledCount, brightness);
  else
    ledStrip.write(ear_colors, ledCount, 0);

}




int x    = 2;
int pass = 0;
int seed = 0;
//This method randomly blinks in different ways
void prashants_eye_loop(){
  seed = random(1,4);

  for (int i = 0;i<40;i++)
  { 
    matrix.fillScreen(0);
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
  //pinMode(limitSwitchPin, INPUT_PULLUP);
  matrix.begin();
  matrix.setTextWrap(false);
  matrix.setBrightness(10);
  //initialize_ear_colors();

  nh.initNode();


  //nh.advertiseService(ear_server);

  nh.advertiseService(eye_server);
  //nh.advertise(limit_pub);
}

void loop()
{ 
  writeEye();
  //writeEar();
  //checkLimitSwitch();

  nh.spinOnce();
  delay(10);
}




