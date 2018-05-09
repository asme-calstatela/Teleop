/*
 * Differential Drive
 * 
 * Written by: Josh Saunders
 * Written on: 12/22/2016
 *
 * Modified by: Josh Saunders
 * Modified on: 1/6/2017
 *
 * Modified by: Jeovanny Reyes
 * Modified on: 8/17/17
 * 
 * Modified by: Jeovanny Reyes
 * Modified on: 12/28/17
 *
 * This sketch takes in a geometry_msgs/Twist message
 * and writes it to left and right Servos. This is an
 * adaptation of:
 * www.ros.org/wiki/rosserial_arduino_demos
 * It also sends out information about encoder counts (DC motors coupled with optical encoders) to be used for odometry calculation.
 *
 * Input: geometry_msgs/Twist cmd_msg that corresponds to=
 *        commands to send the differential drive robot
 *
 * Output: Movement based on the geometry_msgs/Twist commands
 *         Encoder ticks for Odometry and for PID controller
 *
 * Subscribers: "cmd_vel" receives a geometry_msg/Twist command
 *
 * Publishers: "left_enc_ticks" for left encoder count
 *             "right_enc_ticks" for right encoder count
 *
 * Direction conventions used:
 *      TRANSLATIONS
 *      ------------
 *              FORWARD: linear.x = 1
 *              REVERSE: linear.x = -1
 *
 *      ROTATIONS
 *      ---------
 *              COUNTER CLOCKWISE: angular.z = 1
 *              CLOCKWISE        : angular.z = -1
 *
 * Constants: 
 *      STOP (float)       : Angle that stops the servo
 *      LEFT_SCALE (float) : Use this value to "tune" the system so that the servos spin at the same rate
 *      RIGHT_SCALE (float): Use this value to "tune" the system so that the servos spin at the same rate
 *      THRESHOLD (float)  : This value to keep the servos from spinning when the joystick is near center
 *      
 * How to run: 
 * - Make sure that 'roscore' is running
 * - Run 'rosrun rosserial_python serial_node.py /dev/ttyACMx' in the terminal.
 *   [Note: Replace 'x' with the port that your device is connected to]
 */

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

#include <Encoder.h> // Added this line 11/1/17
#include <std_msgs/Float32.h> //Added this line 12/28/17

ros::NodeHandle  nh;


const float STOP = 90;  // The stopping point for the servos is 90 degrees
const float LEFT_SCALE = 0.1757; // Original value 15. Higher value, lower speed. Lower value, higher speed.
const float RIGHT_SCALE = 0.1556; // Original value 15
const float THRESHOLD = 0.01;
int TRAN = 11; // Sending a signal to the transistor/relay

long oldPosition_left = -999; // Added this line 11/1/17
long oldPosition_right = -999;

Servo servo_left;
Servo servo_right;
Encoder left_enc(2,3); // Added this line 11/1/17
Encoder right_enc(4,5);

void servo_cb(const geometry_msgs::Twist& cmd_msg){
  // @param direction_left: speed to be written to the Servos

  int direction_left  = STOP;
  int direction_right = STOP;

  float linear = cmd_msg.linear.x;
  float angular = cmd_msg.angular.z;

  if (linear > THRESHOLD || linear < -THRESHOLD) {
    // Go forward
    direction_left = int ( STOP * (1 - LEFT_SCALE * linear) );
    direction_right = int ( STOP * (1 + RIGHT_SCALE * linear) );
    
    Serial.print("Translate\n");
  } else if (angular > THRESHOLD || angular < -THRESHOLD) {
    // Rotate counter clockwise
    direction_left = int ( STOP * (1 + LEFT_SCALE * angular) );
    direction_right = int ( STOP * (1 + RIGHT_SCALE * angular) );
    
    Serial.print("Rotate\n");
  } else {
    // STOP!

    // TODO: engage the brakes here...
    direction_left = STOP;
    direction_right = STOP;
    //digitalWrite(TRAN, LOW); //Activating the relay to diesngage the breaks. Test out!!!
    
    Serial.print("Stop\n");
  }

  servo_left.write(direction_left); //set servo_left angle
  servo_right.write(direction_right); //set servo_right angle
}

ros::Subscriber<geometry_msgs::Twist> sub_cmd_vel("cmd_vel", servo_cb); // Topic used to be "cmd_vel". New topic is "cmd_vel"ard"

std_msgs::Float32 left_msg; // This instantiates the publisher string
std_msgs::Float32 right_msg; 
ros::Publisher left_count( "/left_enc_ticks", &left_msg); // 
ros::Publisher right_count("/right_enc_ticks", &right_msg);

void setup(){
  nh.initNode();
  nh.subscribe(sub_cmd_vel);
  nh.advertise(left_count);
  nh.advertise(right_count);

  servo_left.attach(8); //attach it to pin 8
  servo_right.attach(9); //attach it to pin 9
 
 pinMode(TRAN, OUTPUT);

 
}

void loop(){
  nh.spinOnce();
  
  long newPosition_left = left_enc.read(); // Added this line 11/1/17
  long newPosition_right = right_enc.read();
  
  if (newPosition_left != oldPosition_left && newPosition_right != oldPosition_right) {  // Added this line 11/1/17
    oldPosition_left = newPosition_left;   // Added this line 11/1/17
    oldPosition_right = newPosition_right;
    
    left_msg.data = newPosition_left;
    right_msg.data = newPosition_right;
    
    left_count.publish( &left_msg);
    right_count.publish( &right_msg);

    delay(30);
    
  }
 digitalWrite(TRAN, HIGH); //Activating the relay to diesngage the breaks
 delay(1);
}
