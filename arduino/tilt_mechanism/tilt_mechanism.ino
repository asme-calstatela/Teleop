/*
 * Tilt Mechanism
 * 
 * Written by: Francisco Moxo
 * Written on: 06/20/2017
 *
 * Modified by: Francisco Moxo
 * Modified on: 08/21/2017
 *

 *
 * Input: sensormsgs/Joy joy that corresponds to
 *        commands that move the tilt mechanism
 *
 * Output: tilt based on the buttons pressed on the joystick
 *
 * Subscribers: "joy" receives a sensormsgs/Joy command
 *
 * Publishers: None
 *
 *
 *      
 */

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h>
#include <ros.h>
#include <sensor_msgs/Joy.h>

ros::NodeHandle  nh;




// use only for tilt arduino
// Top Motor
Servo servo_tilt1; //bottom tilt motor
// Bottom Motor
Servo servo_tilt2; //bottom tilt motor





void joydata( const sensor_msgs::Joy& joy){
int tilt1bk =  joy.buttons[1];
int tilt1fwd =  joy.buttons[3];
int tilt2bk =  joy.buttons[0];
int tilt2fwd =  joy.buttons[2];



//Top Motor
    if (tilt1fwd == 1){
    servo_tilt1.write (88); // tilt fordward
    delay(1000);
    servo_tilt1.write(90); // pause tilt
  }

  if (tilt1bk == 1){
    servo_tilt1.write (102); // tilt fordward
    delay(1000);
    servo_tilt1.write(90); // pause tilt;
  }
 // Bottom Motor
  if (tilt2fwd == 1){
    servo_tilt2.write (85); // tilt fordward
    delay(1000);
    servo_tilt2.write(90); // pause tilt
  }

  if (tilt2bk == 1){
    servo_tilt2.write (98); // tilt fordward
    delay(1000);
    servo_tilt2.write(90); // pause tilt;
  }
  
}


ros::Subscriber<sensor_msgs::Joy> sub_joy_button("joy", joydata);

void setup(){
  nh.initNode();
  nh.subscribe(sub_joy_button);


// use only for tilt mechanism  
  servo_tilt1.attach(10); // attach it to pin 10 for tilt
  servo_tilt2.attach(11); // attach it to pin 10 for tilt
  servo_tilt2.write (90); // tilt fordward
  servo_tilt1.write (90); // tilt fordward




}

void loop(){
  nh.spinOnce();
  delay(1);
}
