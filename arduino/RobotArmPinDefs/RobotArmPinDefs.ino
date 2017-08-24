#include<Servo.h>

/*
 * Robot Arm Code
 * 
 * Created 7-8-2017 by Matthew Tran
 */

//5 DOF Robot Arm Joints' Names
  //Shoulder Pitch - forward back
  //Shoulder Yaw - left right
  //Elbow
  //Wrist Pitch - up down
  //Wrist Roll - rotational
//Preferred Coordinate System is clockwise,right,and upwards are positive

//feel free to change variable names

//Shoulder Pitch
#define spMotorPin 3 //motor pin
Servo spMotor; //be careful when moving motor since esc is sensitive
const int spMotorMin = 30; //min servo.write value; values can be reset check manual
const int spMotorMax = 150; //max servo.write value
const int spMotorNeutral = 90; //no movement
#define spEncoderA A8 //2 channels per encoder for positioning
#define spEncoderB A9
#define spEncoderH A7 //homing pin
const int spEncoderAMin = ; //these values need to be determined
const int spEncoderAMax = ; //mid "0" value can just be average of min and max
const int spEncoderBMin = ; //these values need to be determined
const int spEncoderBMax = ;
const int spEncoderHTrigger = ; //below this homing pin is "triggered"
double spPos = 0; //position of joint, about ~200° in both directions from pointing down
const double spPosMin = ; //experimentally determined
const double spPosMax = ;
const double spPosH = ; //position to set to when homed

//Shoulder Yaw
#define syMotorPin 2
Servo syMotor; //be careful when moving motor since esc is sensitive
const int syMotorMin = 30;
const int syMotorMax = 150;
const int syMotorNeutral = 90;
#define syEncoderA A5
#define syEncoderB A6
#define syEncoderH A4
const int syEncoderAMin = ;
const int syEncoderAMax = ;
const int syEncoderBMin = ;
const int syEncoderBMax = ;
const int syEncoderHTrigger = ;
double syPos = 0; //position of joint, about ~90° in both directions from pointing down
const double syPosMin = ;
const double syPosMax = ;
const double syPosH = ;

//Elbow
#define eMotorPin 1
Servo eMotor; //be careful when moving motor since esc is sensitive
const int eMotorMin = 0;
const int eMotorMax = 180;
const int eMotorNeutral = 90;
#define eEncoderA A2
#define eEncoderB A3
#define eEncoderH A1
const int eEncoderAMin = ;
const int eEncoderAMax = ;
const int eEncoderBMin = ;
const int eEncoderBMax = ;
const int eEncoderHTrigger = ;
double ePos = 0; //position of joint, about ~110° in both directions from pointing down
const double ePosMin = ;
const double ePosMax = ;
const double ePosH = ;

//Wrist Pitch
#define wpMotorPulse 8 //pulse pin for stepper, currently set at 200steps/rev (connected to 1:50 gearbox)
#define wpMotorDir 9 //direction pin for stepper
const int wpMotorMax = 3000; //max step pulse delay (microseconds) this one can be changed
const int wpMotorMin = 500; //min step pulse delay (microseconds) delay is half of full cycle length (any faster and it might bind)
#define wpEncoderA A0
#define wpEncoderB A22
#define wpEncoderH A21
const int wpEncoderAMin = ;
const int wpEncoderAMax = ;
const int wpEncoderBMin = ;
const int wpEncoderBMax = ;
const int wpEncoderHTrigger = ;
double wpPos = 0; //position of joint, about ~105° in both directions from pointing down
const double wpPosMin = ;
const double wpPosMax = ;
const double wpPosH = ;

//Wrist Pitch
#define wpMotorPulse 11 //pulse pin for stepper, currently set at 200steps/rev (connected to 1:50 gearbox)
#define wpMotorDir 12 //direction pin for stepper
const int wpMotorMax = 3000; //max step pulse delay (microseconds) this one can be changed
const int wpMotorMin = 500; //min step pulse delay (microseconds) delay is half of full cycle length (any faster and it might bind)
#define wpEncoderA A19
#define wpEncoderB A20
#define wpEncoderH A18
const int wpEncoderAMin = ;
const int wpEncoderAMax = ;
const int wpEncoderBMin = ;
const int wpEncoderBMax = ;
const int wpEncoderHTrigger = ;
double wpPos = 0; //position of joint, should be unlimited rotation depending on wires
const double wpPosMin = ;
const double wpPosMax = ;
const double wpPosH = ;

void setup() {
  Serial.begin(250000);
  //analogReadRes(12); //uncomment to increase resolution, but will decrease stability
  analogReadAveraging(32); //stabilizes readings
  spMotor.attach(spMotorPin);
  syMotor.attach(syMotorPin);
  eMotor.attach(eMotorPin);
  spMotor.write(spMotorNeutral);
  syMotor.write(syMotorNeutral);
  eMotor.write(eMotorNeutral);
  //might want to use AccelStepper library for steppers
}

void loop() {

}
