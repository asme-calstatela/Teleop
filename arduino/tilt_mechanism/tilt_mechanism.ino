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

//int buttontilt1 = joy.button[1]; // Yellow button
//int buttontilt2 = joy.button[3]; //   button

//Top Motor
    if (buttontitl1 == 1 && buttontilt2 ==0){ // Yellow buttong pressed and black code
    servo_tilt1.write (85); // tilt fordward. Originally 88
    delay(1000);
    servo_tilt1.write (88); // tilt fordward. Originally 88. New Value 80
    delay(500);
    servo_tilt1.write(90); // pause tilt
  }

  if (tilt1bk == 1){
<<<<<<< HEAD
    servo_tilt1.write (95); // tilt backward . Originally 102
    delay(1000);
=======
    servo_tilt1.write (102); // tilt backward. Originally 102. New Value 110.
    delay(500);
>>>>>>> cc5a1efdad832788a0cd02adbfffaeaccf0b3bbe
    servo_tilt1.write(90); // pause tilt;
  }
 // Bottom Motor
  if (tilt2fwd == 1){
<<<<<<< HEAD
    servo_tilt2.write (85); // tilt fordward. Originally 85
    delay(1000);
=======
    servo_tilt2.write (85); // tilt fordward. Originally 85. New Value 75.
    delay(500);
>>>>>>> cc5a1efdad832788a0cd02adbfffaeaccf0b3bbe
    servo_tilt2.write(90); // pause tilt
  }

  if (tilt2bk == 1){
<<<<<<< HEAD
    servo_tilt2.write (95); // tilt backward. Originally 98
    delay(1000);
=======
    servo_tilt2.write (9); // tilt backward. Originally 98. New Value 95
    delay(500);
>>>>>>> cc5a1efdad832788a0cd02adbfffaeaccf0b3bbe
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
