/*
   Cytron Tilt Mechanism

   Written by: Francisco Moxo
   Written on: 05/9/2018

   Modified by: Francisco Moxo
   Modified on: 05/9/2018

   Input: sensormsgs/Joy joy that corresponds to
          commands that move the tilt mechanism

   Output: tilt based on the buttons pressed on the joystick

   Subscribers: "joy" receives a sensormsgs/Joy command

   Publishers: None

*/
#include <ros.h>
#include <sensor_msgs/Joy.h>

#define dir_1 6 // direction pin
#define pwm_1 5 // PWM pin MUST be PWM PIN
#define dir_2 7 // direction pin
#define pwm_2 4 // PWM pin MUST be PWM PIN
#define pot A0 //potentiometer

ros::NodeHandle  nh;

int maxPWM = 255; // The maximum PWM signal range from  0-255
int maxCtrlFWD = 1023; // max value for Controller Fordward can be 1023 or positive number
int midCtrl = 511.5; // mid-location of Controller reading, might be 0 or 511.5
int minCtrlBWD = 0; // min- value contoller gives in backwards direction can be a negative number or 0
int deadZone = 11.5;// the amount of tolarance befor motordriver acts

int reading = 0 ; //intial reading from controller
int pwm_value = 0; // PWM value sent to Cytron

void joydata( const sensor_msgs::Joy& joy) {
  //controls the direction the motor
  int buttontilt1 = 1; //get reading from controller button
  int buttontilt2 = 1; //get reading from controller button
//Low Tilt  
  if (buttontilt1 ==1 && buttontilt2 ==0){
  reading = analogRead(pot); // reading is equal to the value of telop controller
  // Direction 1
  if (reading < midCtrl - deadZone) {
    //    Serial.print(reading);
    //    Serial.print(" ");
    pwm_value = map(reading, midCtrl , minCtrlBWD, 0, 255);
    digitalWrite(dir_1, LOW);
    analogWrite(pwm_1, pwm_value);
  }
  // Direction 2
  else if (reading > midCtrl + deadZone) {
    pwm_value = map(reading, midCtrl, maxCtrlFWD , 0, maxPWM);
    digitalWrite(dir_1, HIGH);
    analogWrite(pwm_1, pwm_value);
  }
  // DeadZone do nothing
  else {
    pwm_value = 0;
    analogWrite(pwm_1, pwm_value);
  }
  //  Serial.println(pwm_value); //Display the value of PWM
  delay(5);
}
// Upper Tilt
  if (buttontilt2 ==1 && buttontilt1 ==0){
  reading = analogRead(pot); // reading is equal to the value of telop controller
  // Direction 1
  if (reading < midCtrl - deadZone) {
    //    Serial.print(reading);
    //    Serial.print(" ");
    pwm_value = map(reading, midCtrl , minCtrlBWD, 0, 255);
    digitalWrite(dir_2, LOW);
    analogWrite(pwm_2, pwm_value);
  }
  // Direction 2
  else if (reading > midCtrl + deadZone) {
    pwm_value = map(reading, midCtrl, maxCtrlFWD , 0, maxPWM);
    digitalWrite(dir_2, HIGH);
    analogWrite(pwm_2, pwm_value);
  }
  // DeadZone do nothing
  else {
    pwm_value = 0;
    analogWrite(pwm_2, pwm_value);
  }
  //  Serial.println(pwm_value); //Display the value of PWM
  delay(5);
}
}

ros::Subscriber<sensor_msgs::Joy> sub_joy_button("joy", joydata);

void setup() {
  pinMode(pwm_1, OUTPUT);
  pinMode(dir_1, OUTPUT);
  pinMode(pot, INPUT);
  Serial.begin(9600); //I am using Serial Monitor instead of LCD display

  nh.initNode();
  nh.subscribe(sub_joy_button);
}


void loop() {
  nh.spinOnce();
}
