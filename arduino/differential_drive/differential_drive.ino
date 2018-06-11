 /*
 * Differential Drive
 * 
 * Written by: Josh Saunders
 * Written on: 12/22/2016
 *
 * Modified by: Josh Saunders
 * Modified on: 1/6/2016
 *
 * Modified by: Jeovanny Reyes
 * Modified on: 8/17/17
 *
 * This sketch takes in a geometry_msgs/Twist message
 * and writes it to left and right Servos. This is an
 * adaptation of:
 * www.ros.org/wiki/rosserial_arduino_demos
 *
 * Input: geometry_msgs/Twist cmd_msg that corresponds to
 *        commands to send the differential drive robot
 *
 * Output: Movement based on the geometry_msgs/Twist commands
 *
 * Subscribers: "cmd_vel" receives a geometry_msg/Twist command
 *
 * Publishers: None
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
#include <std_msgs/Float32.h>
#include <sensor_msgs/Joy.h>

ros::NodeHandle  nh;
std_msgs::Float32 left_wheel_msg;
std_msgs::Float32 right_wheel_msg;
ros::Publisher motor_signal1("motor_signal1", &left_wheel_msg);
ros::Publisher motor_signal2("motor_signal2", &right_wheel_msg);

const float STOP = 90;  // The stopping point for the servos is 90 degrees
const float LEFT_SCALE = 0.0888; // Original value 15. Higher value, lower speed. Lower value, higher speed.
const float RIGHT_SCALE = 0.0888; // Original value 15
const float THRESHOLD = 0.01;
int TRAN = 11; // Sending a signal to the transistor/relay

Servo servo_left;
Servo servo_right;

int state_run = 0; // Variable to tell if trigger is pressed


void joydata(const sensor_msgs::Joy& joy){
  int hold_buttval = joy.buttons[0]; 
  if (joy.buttons[0] == 1 && joy.buttons[1] == 0 && joy.buttons[3] == 0) {
     state_run = 1; // This will activate to get robot to move
  }
  else {
    state_run = 0; // Robot will not drive!!!
    }
}
void servo_cb(const geometry_msgs::Twist& cmd_msg){
  // @param direction_left: speed to be written to the Servos

  int direction_left  = STOP;
  int direction_right = STOP;

  float linear = cmd_msg.linear.x;
  float angular = cmd_msg.angular.z;
  if (state_run == 1) {
    if (linear > THRESHOLD || linear < -THRESHOLD) {
      // Go forward
      direction_left = int ( STOP * (1 - LEFT_SCALE * linear) );
      direction_right = int ( STOP * (1 + RIGHT_SCALE * linear) );
      
      left_wheel_msg.data = direction_left;
      right_wheel_msg.data = direction_right;
      
   } else if (angular > THRESHOLD || angular < -THRESHOLD) {
    // Rotate counter clockwise
      direction_left = int ( STOP * (1 + LEFT_SCALE * angular) );
      direction_right = int ( STOP * (1 + RIGHT_SCALE * angular) );

      left_wheel_msg.data = direction_left;
      right_wheel_msg.data = direction_right;
    
    } else {
      // STOP!
      direction_left = STOP;
      direction_right = STOP;

      left_wheel_msg.data = direction_left;
      right_wheel_msg.data = direction_right;
    
    }

  servo_left.write(direction_left); //set servo_left angle
  servo_right.write(direction_right); //set servo_right angle
  
  motor_signal1.publish( &left_wheel_msg);
  motor_signal2.publish( &right_wheel_msg);
    
  }
  else {
    direction_left = STOP;
    direction_right = STOP;
    left_wheel_msg.data = direction_left;
    right_wheel_msg.data = direction_right;
    servo_left.write(direction_left); //set servo_left angle
    servo_right.write(direction_right); //set servo_right angle
    motor_signal1.publish( &left_wheel_msg);
    motor_signal2.publish( &right_wheel_msg);
    }
}

ros::Subscriber<geometry_msgs::Twist> sub_cmd_vel("cmd_vel", servo_cb);
ros::Subscriber<sensor_msgs::Joy> sub_joy_button("joy", joydata);


void setup(){
  nh.initNode();
  nh.subscribe(sub_cmd_vel);
  nh.subscribe(sub_joy_button);
  nh.advertise(motor_signal1);
  nh.advertise(motor_signal2);

  servo_left.attach(8); //attach it to pin 8
  servo_right.attach(9); //attach it to pin 9
 
 pinMode(TRAN, OUTPUT);
}

void loop(){
  nh.spinOnce();
 
 digitalWrite(TRAN, HIGH); //Activating the relay to diesngage the breaks
 delay(1);
}
