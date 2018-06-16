/********************************************************
 * Written By: Jeovanny Reyes 
 * Created On: 4/18/18
 * 
 * Purpose: To control the speed of the left and right wheel
 * 
 * This is an
 * adaptation of:
 * www.ros.org/wiki/rosserial_arduino_demos
 * and some code is taken from the "PID Basic" Example and the Encoder "Two_Knobs" Example
 * 
 *  Error Calculation: Error = Setpoint - Input
 *  Setpoint: Velocity of Left Wheel
 *  Input: Velocity coming from Right Wheel
 *  Output: Velocity calculated outputed to right wheel to match left wheel
 ********************************************************/

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <PID_v1.h>
#include <Servo.h>
#include <ros.h>
#include <geometry_msgs/Twist.h> // These are our inputs
#include <std_msgs/Float32.h> // Velocity Info From Odom code
#include <sensor_msgs/Joy.h> // Used for info about joystick buttons

ros::NodeHandle nh;

/////////////////////////////////////////////////// PUBLISHERS //////////////////////////////////////////////
std_msgs::Float32 left_wheel_msg;
std_msgs::Float32 right_wheel_msg;
ros::Publisher motor_signal1("motor_signal1", &left_wheel_msg); // Monitoring commands being sent to motor
ros::Publisher motor_signal2("motor_signal2", &right_wheel_msg);
/////////////////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////// VARIABLES & OBJECTS /////////////////////////////////////////
const float STOP = 90; // The stopping point for the servos is 90 degrees
const float THRESHOLD = 0.01; // Serves as a deadzone for joystick sensitivity
const float LEFT_SCALE = 0.48; // Original value 15
int TRAN = 11; //Sending a signal to avtivate/disactivate transistor relay for brakes

Servo servo_left; // Object for Left DC Motor
Servo servo_right; // Object for Right DC Motor

int state_run = 0; //Variable to tell if trigger is pressed to have Melo move!

double Setpoint, Input, Output; //Define Variables we'll be connecting to
double Linear_Setpoint, Angular_Setpoint;

double Kp=1.2, Ki=5, Kd=1; //Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT); // Direct is for negative feedback. Input gets stored as "Linear_Input" or "Angular_Setpoint"
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////// CALLBACKS /////////////////////////////////////////////////
void joydata(const sensor_msgs::Joy& joy){
  int hold_buttval = joy.buttons[0]; 
  if (joy.buttons[0] == 1 && joy.buttons[1] == 0 && joy.buttons[3] == 0) {
     state_run = 1; // This will activate to get robot to move
  }
  else {
    state_run = 0; // Robot will not drive!!!
    }
}

void left_lin_vel_cb(const std_msgs::Float32& left_vel_msg){
    Setpoint = left_vel_msg.data;
  }

void right_lin_vel_cb(const std_msgs::Float32& right_vel_msg){
    Input = right_vel_msg.data;
  }

void servo_cb(const geometry_msgs::Twist& cmd_msg){

  int direction_left  = STOP;
  int direction_right = STOP;

  double linear = cmd_msg.linear.x;
  double angular = cmd_msg.linear.z;

  if (linear !=0 && linear > THRESHOLD && linear < -THRESHOLD){
    Setpoint = linear; // Forward or Backwards commands [-0.01 to 0.01]
  }
  else if{angular !=0 && angular > THRESHOLD && angular < -THRESHOLD){
    Setpoint = angular; // Rotate Left or Right commands [-0.01 to 0.01] 
  }
  
  if (state_run == 1) { // If trigger on joystick is pressed
    if (linear > THRESHOLD || linear < -THRESHOLD) {
      // Go forward or backwards
      direction_left = int ( STOP * (1 - LEFT_SCALE * linear ) );
      direction_right = int ( STOP * (1 + Output * linear) );
      
      left_wheel_msg.data = direction_left;
      right_wheel_msg.data = direction_right;
      
   } else if (angular > THRESHOLD || angular < -THRESHOLD) {
      // Rotate counter clockwise or clockwise
      direction_left = int ( STOP * (1 + LEFT_SCALE * angular) ); 
      direction_right = int ( STOP * (1 + Output * angular) );

      left_wheel_msg.data = direction_left;
      right_wheel_msg.data = direction_right;
    
    } else {
      direction_left = STOP; // OUTPUT
      direction_right = STOP; // OUTPUT

      left_wheel_msg.data = direction_left;
      right_wheel_msg.data = direction_right;
    
    }

  servo_left.write(direction_left); //set servo_left speed
  servo_right.write(direction_right); //set servo_right speed
  
  motor_signal1.publish( &left_wheel_msg);
  motor_signal2.publish( &right_wheel_msg);
    
  }
  else { // Trigger on joystick is not pressed. Melo cant move!
    direction_left = STOP; // OUTPUT
    direction_right = STOP; // OUTPUT
    left_wheel_msg.data = direction_left;
    right_wheel_msg.data = direction_right;
    servo_left.write(direction_left); //set servo_left speed
    servo_right.write(direction_right); //set servo_right speed 
    motor_signal1.publish( &left_wheel_msg);
    motor_signal2.publish( &right_wheel_msg);
    }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////// SUBSCRIBERS ///////////////////////////////////////////////
ros::Subscriber<geometry_msgs::Twist> sub_cmd_vel("cmd_vel", servo_cb); // Contains -1 to 1 info
ros::Subscriber<sensor_msgs::Joy> sub_joy_button("joy", joydata); // Contains 0 or 1 about button state
ros::Subscriber<std_msgs::Float32> sub_left_lin_vel("left_lin_vel", left_lin_vel_cb); // Contains linear velocity info of left
ros::Subscriber<std_msgs::Float32> sub_right_lin_vel("right_lin_vel", right_lin_vel_cb); // Contains linear velocity info of right
////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup()
{
  nh.initNode();
  nh.subscribe(sub_cmd_vel); // Subscribing to joystick movements
  nh.subscribe(sub_joy_button); // Subscribing to state info about joystick buttons
  nh.subscribe(sub_left_lin_vel); // Subscribing to velocity of left wheel
  nh.subscribe(sub_right_lin_vel); // Subscribing to velocity of right wheel
  nh.advertise(motor_signal1); // Info about left motor command
  nh.advertise(motor_signal2); // Infor about right motor command
   
  //initialize the variables we're linked to
  Setpoint = 1; // For both left and right wheel

  servo_left.attach(8); // Attach it to pin 8
  servo_right.attach(9); // Attach it to pin 9

  myPID.SetMode(AUTOMATIC); // Turn the PID on
  pinMode(TRAN, OUTPUT); // Connect to Transistor Relay
}

void loop()
{
  digitalWrite(TRAN, HIGH); // Activating the relay to disengage the breaks!
  nh.spinOnce(); // Handles ALL Callbacks
  myPID.Compute(); // This calculates the error and adjusted output
  //nh.spinOnce(); // Handles ALL Callbacks!
}


