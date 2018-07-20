/*
 * Tilt Mechanism
 *
 * Written by: Francisco Moxo
 * Written on: 06/20/2017
 *
 * Modified by: Francisco Moxo
 * Modified on: 08/21/2017
 *
 * Modified by Adryel Arizaga
 * Modified on: 05/09/2018
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

int maximumVal = 1;
int minimumVal = -1;

// use only for tilt arduino
// Top Motor
Servo servo_tilt1; //bottom tilt motor
// Bottom Motor
Servo servo_tilt2; //bottom tilt motor


void joydata( const sensor_msgs::Joy& joy){
    int yellowButton =  joy.buttons[1];
    int blackButton =  joy.buttons[3];
    float joyYAxis = joy.axes[1];

    // Guard against inputs greater or less than what is expected from the joystick
    if (joyYAxis > maximumVal) {
        maximumVal = joyYAxis;
    }
    if (joyYAxis < minimumVal) {
        minimumVal = joyYAxis;
    }

    //Top Motor
    int writeValue = mapAxis(joyYAxis, minimumVal, maximumVal);

    if (yellowButton == 1){
        servo_tilt1.write (writeValue); // tilt fordward. Originally 88. New Value 80
        // servo_tilt1.write(90); // pause tilt
    }

     // Bottom Motor
    else if (blackButton == 1){
        servo_tilt2.write (writeValue); // tilt fordward. Originally 88. New Value 80
        // servo_tilt2.write(90); // pause tilt
    }
    else {
        servo_tilt1.write(90);
        servo_tilt2.write(90);
    }

}

long mapAxis(float x, int minIn, int maxIn)
{
  return (long)((x - minIn) * (30) / (maxIn - minIn)) + 75;
}

ros::Subscriber<sensor_msgs::Joy> sub_joy_button("joy", joydata);

void setup(){
  nh.initNode();
  nh.subscribe(sub_joy_button);


// use only for tilt mechanism
  servo_tilt1.attach(10); // attach it to pin 10 for tilt
  servo_tilt2.attach(12); // attach it to pin 11 for tilt
  servo_tilt2.write (90); // pause tilt
  servo_tilt1.write (90); // pause tilt




}

void loop(){
  nh.spinOnce();
  delay(1);
}
