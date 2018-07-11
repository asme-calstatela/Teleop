/*
 * Written By: Jeovanny Reyes
 * 
 * Objective: To control neck servos (3) through joystick
 * 
 * To run:
 *        Run 'roscore'
 *        Run 'rosrun rosserial_python serial_node.py /dev/ttyACMx'
 *        where x is the port the arduino is connected to
 */

#include <ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32.h>
#include <Servo.h>

ros::NodeHandle nh;

///////////////////////////// PUBLISHER ////////////////////
std_msgs::Float32 first_servo_msg;
std_msgs::Float32 second_servo_msg;
ros::Publisher servo_pos1("servo_pos1", &first_servo_msg);
ros::Publisher servo_pos2("servo_pos2", &second_servo_msg);
///////////////////////////////////////////////////////////////

Servo servo1, servo2, servo3;

int pos1 = 90;
int pos2 = 90;
int pos3 = 90;

int new_pos1 = 90;
int new_pos2 = 90;
int new_pos3 = 90;

int old_pos1 = 90;
int old_pos2 = 90;
int old_pos3 = 90;

void joydata(const sensor_msgs::Joy& joy){
  int joy_butt = joy.buttons[2]; // Trigger to move servos 1 and 2
  float joy_fb = joy.axes[1]; // forward and backward
  float joy_lr = joy.axes[0]; // left and right
  float joy_servo3 = joy.axes[4]; // small black joystick. Up is +1
  
  if (joy_butt == 1){
       new_pos1 = pos1 + (pos1*joy_fb);
       new_pos2 = pos2 + (pos2*joy_lr);

       first_servo_msg.data = new_pos1;
       second_servo_msg.data = new_pos2;

       servo_pos1.publish( &first_servo_msg);
       servo_pos2.publish( &second_servo_msg);
      
       servo1.write(new_pos1);
       servo2.write(new_pos2);

       old_pos1 = new_pos1;
       old_pos2 = new_pos2;
  }

  else if(joy_servo3 > 0){
      new_pos3 = pos3 + (pos3*joy_fb);

      servo3.write(new_pos3);

      old_pos3 = new_pos3;
  }
  else {
     servo1.write(old_pos1);
     servo2.write(old_pos2);
     servo3.write(old_pos3);

     first_servo_msg.data = new_pos1;
     second_servo_msg.data = new_pos2;

     servo_pos1.publish( &first_servo_msg);
     servo_pos2.publish( &second_servo_msg);
  }
}


ros::Subscriber<sensor_msgs::Joy> sub_joy_button("joy", joydata);

void setup() {
nh.initNode();
nh.subscribe(sub_joy_button);
nh.advertise(servo_pos1);
nh.advertise(servo_pos2);

servo1.attach(9, 500, 2500);
servo2.attach(10, 500, 2500);
servo3.attach(11, 500, 2500);
servo1.write(pos1);
servo2.write(pos2);
servo3.write(pos3);
}

void loop() {
  nh.spinOnce();
  delay(1);
}
