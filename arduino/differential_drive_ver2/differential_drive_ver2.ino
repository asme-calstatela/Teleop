/********************************************************
 * Written By: Jeovanny Reyes 
 * Created On: 4/18/18
 * 
 * Purpose: To control the speed of the left and right wheel
 * 
 * This is an
 * adaptation of:
 * www.ros.org/wiki/rosserial_arduino_demos
 * 
 * To run:
 *        Run 'roscore'
 *        Run 'rosrun rosserial_python serial_node.py /dev/ttyACMx'
 * This method uses dynamic reconfiguration
 ********************************************************/

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h>
#include <ros.h>
#include <geometry_msgs/Twist.h> // These are our inputs that came from joystick
#include <std_msgs/Float32.h> // Velocity Info From Odom code
#include <sensor_msgs/Joy.h> // Used for info about joystick buttons
#include <dynamic_reconfigure/Config.h> // Updated values for velocity scaling
//#include <rosserial_arduino/Test.h>

ros::NodeHandle nh;
//using rosserial_arduino::Test;

////////////////////////////////// SERVICES //////////////////////////////////////////////////////////////
//ros::ServiceClient<Test::Request, Test::Response> client("/diff_drive_server_node"); // then try /update_parameters
//////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////// PUBLISHERS //////////////////////////////////////////////
std_msgs::Float32 left_wheel_msg;
std_msgs::Float32 right_wheel_msg;
ros::Publisher motor_signal1("motor_signal1", &left_wheel_msg); // Monitoring commands being sent to motor
ros::Publisher motor_signal2("motor_signal2", &right_wheel_msg);
/////////////////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////// VARIABLES & OBJECTS /////////////////////////////////////////
const float STOP = 90; // The stopping point for the servos is 90 degrees
const float THRESHOLD = 0.01; // Serves as a deadzone for joystick sensitivity
float LEFT_SCALE = 0.48; // To tune left wheel going forward [Dyn Con Updates
float RIGHT_SCALE = 0.48; // To tune right wheel going forward [Dyn Con Updates]
const float LEFT_SCALE_BCKWD = 0.48; // To tune left wheel going backward
const float RIGHT_SCALE_BCKWD = 0.48; // To tune right wheel going backward
int TRAN = 11; //Sending a signal to avtivate/disactivate transistor relay for brakes

Servo servo_left; // Object for Left DC Motor
Servo servo_right; // Object for Right DC Motor

int state_run = 0; //Variable to tell if trigger is pressed to have Melo move!

//////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////// CALLBACKS /////////////////////////////////////////////////
//void vel_scales(const 

void joydata(const sensor_msgs::Joy& joy){
  int hold_buttval = joy.buttons[0]; 
  if (joy.buttons[0] == 1 && joy.buttons[1] == 0 && joy.buttons[3] == 0) {
     state_run = 1; // This will activate to get robot to move
  }
  else {
    state_run = 0; // Robot will not drive!!!
    }
}

void vel_scale_cb(const dynamic_reconfigure::Config& scale_val){
   //float LEFT_SCALE = scale_val.doubles.value; // Check to see if this is correct
   float RIGHT_SCALE = scale_val.doubles_length; // Check to see if this correct
   //left_wheel_msg.data = LEFT_SCALE; 
   //left_wheel_msg.data = RIGHT_SCALE;

   //motor_signal1.publish( &left_wheel_msg); // left scaling
   //motor_signal2.publish( &right_wheel_msg); // right scaling
}

void servo_cb(const geometry_msgs::Twist& cmd_msg){

  int direction_left  = STOP;
  int direction_right = STOP;

  double linear = cmd_msg.linear.x;
  double angular = cmd_msg.angular.z;
  
  if (state_run == 1) { // If trigger on joystick is pressed
    if (linear > THRESHOLD || linear < -THRESHOLD) {
      // Go forward or backwards
      direction_left = int ( STOP * (1 - LEFT_SCALE * linear ) );
      direction_right = int ( STOP * (1 + RIGHT_SCALE * linear) );
      
      left_wheel_msg.data = direction_left;
      right_wheel_msg.data = direction_right;
      
   } else if (angular > THRESHOLD || angular < -THRESHOLD) {
      // Rotate counter clockwise or clockwise
      direction_left = int ( STOP * (1 + LEFT_SCALE * angular) ); 
      direction_right = int ( STOP * (1 + RIGHT_SCALE * angular) );

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
//ros::Subscriber<std_msgs::Float32> sub_left_lin_vel("left_lin_vel", left_lin_vel_cb); // Contains linear velocity info of left
//ros::Subscriber<std_msgs::Float32> sub_right_lin_vel("right_lin_vel", right_lin_vel_cb); // Contains linear velocity info of right
ros::Subscriber<dynamic_reconfigure::Config> sub_vel_scale("diff_drive_server_node/parameter_updates", vel_scale_cb); //Contains updated velocity scales
////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup()
{
  nh.initNode();
  //nh.serviceClient(client);
  nh.subscribe(sub_cmd_vel); // Subscribing to joystick movements
  nh.subscribe(sub_joy_button); // Subscribing to state info about joystick buttons
  nh.subscribe(sub_vel_scale); // Dyanmically reconfigured velcity values
  //nh.subscribe(sub_left_lin_vel); // Subscribing to velocity of left wheel
  //nh.subscribe(sub_right_lin_vel); // Subscribing to velocity of right wheel
  nh.advertise(motor_signal1); // Info about left motor command
  nh.advertise(motor_signal2); // Infor about right motor command
  //while(!nh.connected()) nh.spinOnce();

  servo_left.attach(8); // Attach it to pin 8
  servo_right.attach(9); // Attach it to pin 9

  pinMode(TRAN, OUTPUT); // Connect to Transistor Relay
}

void loop()
{
  //Test::Request req; // May not need this
  //Test::Response res; 
  //req.input = LEFT_SCALE; // Dont need this
  //client.call(req,res);
  //client.call(res);
  //LEFT_SCALE= res.output
  //str_msg.data = res.output; //May not need this
  
  digitalWrite(TRAN, HIGH); // Activating the relay to disengage the breaks!
  nh.spinOnce(); // Handles ALL Callbacks
  
}


