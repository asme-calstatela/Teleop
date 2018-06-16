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
const float VEL_THRESHOLD = 0.01; // Serves as a deadzone for joystick sensitivity
int TRAN = 11; //Sending a signal to avtivate/disactivate transistor relay for brakes

Servo servo_left; // Object for Left DC Motor
Servo servo_right; // Object for Right DC Motor

int state_run = 0; //Variable to tell if trigger is pressed to have Melo move!

double Setpoint, Input, Output; //Define Variables we'll be connecting to
double Linear_Setpoint, Angular_Setpoint;

double Kp=2, Ki=5, Kd=1; //Specify the links and initial tuning parameters
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

  Linear_Setpoint = cmd_msg.linear.x // Linear command always changing
  linear = cmd_msg.linear.x
  Angular_Setpoint = cmd.msg.linear.z // Angular command always changing
  angular = cmd_msg.linear.z
  
  if (state_run == 1) { // If trigger on joystick is pressed
    if (Linear_Setpoint > THRESHOLD || Linear_Setpoint < -THRESHOLD) {
      // Go forward or backwards
      Setpoint = Linear_Setpoint; // Will be a positive or negative value
      direction_left = int ( STOP * (1 - LEFT_SCALE * linear) ); // OUTPUT
      direction_right = int ( STOP * (1 + RIGHT_SCALE * linear) ); // OUTPUT
      
      left_wheel_msg.data = direction_left;
      right_wheel_msg.data = direction_right;
      
   } else if (Angular_Setpoint > THRESHOLD || Angular_Setpoint < -THRESHOLD) {
      // Rotate counter clockwise or clockwise
      Setpoint = Angular_Setpoint; // Will be a positive or negative value
      direction_left = int ( STOP * (1 + LEFT_SCALE * angular) ); // OUTPUT
      direction_right = int ( STOP * (1 + RIGHT_SCALE * angular) ); // OUTPUT

      left_wheel_msg.data = direction_left;
      right_wheel_msg.data = direction_right;
    
    } else {
      // STOP!
      Setpoint = Linear_Setpoint; // Will be zero since joystick does not move
      direction_left = STOP; // OUTPUT
      direction_right = STOP; // OUTPUT

      left_wheel_msg.data = direction_left;
      right_wheel_msg.data = direction_right;
    
    }

  servo_left.write(direction_left); //set servo_left speed [OUTPUT]
  servo_right.write(direction_right); //set servo_right speed [OUTPUT]
  
  motor_signal1.publish( &left_wheel_msg);
  motor_signal2.publish( &right_wheel_msg);
    
  }
  else { // Trigger on joystick is not pressed. Melo cant move!
    Setpoint = Linear_Setpoint; // Input signal will be zero since trigger is not pressed!
    direction_left = STOP; // OUTPUT
    direction_right = STOP; // OUTPUT
    left_wheel_msg.data = direction_left;
    right_wheel_msg.data = direction_right;
    servo_left.write(direction_left); //set servo_left speed [OUTPUT]
    servo_right.write(direction_right); //set servo_right speed [OUTPUT]
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
  nh.subscrbe(sub_cmd_vel); // Subscribing to joystick movements
  nh.subscribe(sub_joy_button); // Subscribing to state info about joystick buttons
  nh.subscribe(sub_left_lin_vel); // Subscribing to velocity of left wheel
  nh.subscribe(sub_right_lin_vel); // Subscribing to velocity of right wheel
  nh.advertise(motor_signal1); // Info about left motor command
  nh.advertise(motor_signal2); // Infor about right motor command
   
  //initialize the variables we're linked to
  Linear_Setpoint = cmd_msg.linear.x  // Coming from joystick! [analog]
  Setpoint = Linear_Setpoint; // Setting Linear Velocity command as first input
  Angular_Setpoint = cmd_msg.angular.z // Coming from joystick! [analog]
  Setpoint = 100; // Will be based on left wheel as reference

  servo_left.attach(8); // Attach it to pin 8
  servo_right.attach(9); // Attach it to pin 9

  myPID.SetMode(AUTOMATIC); // Turn the PID on
  pinMode(TRAN, OUTPUT); // Connect to Transistor Relay
}

void loop()
{
  nh.spinOnce(); // Handles ALL Callbacks!

  digitalWrite(TRAN, HIGH) // Activating the relay to disengage the breaks!
  myPID.Compute(); // This calculates the error and 
  analogWrite(PIN_OUTPUT, Output);
}


