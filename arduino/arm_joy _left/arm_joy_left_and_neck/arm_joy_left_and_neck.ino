#include <Servo.h>
#include <ros.h>
#include <sensor_msgs/Joy.h>
#include <Arduino.h>
#include "DRV8825.h"

ros::NodeHandle nh;

int maximumval= 1;
int minimumval= -1;

int pos1 = 1500;
int pos2 = 1500;
int pos3 = 1500;

int pos1_neck = 90;
int pos2_neck = 90;
int pos3_neck = 90;

int new_pos1 = 1500;
int new_pos2 = 1500;
int new_pos3 = 1500;

int new_pos1_neck = 90;
int new_pos2_neck = 90;
int new_pos3_neck = 90;

int old_pos1 = 1500;
int old_pos2 = 1500;
int old_pos3 = 1500;

int old_pos1_neck = 90;
int old_pos2_neck = 90;
int old_pos3_neck = 90;

#define MOTOR_STEPS 200
#define RPM 120

#define DIR 5
#define STEP 6
#define MODE0 9
#define MODE1 8
#define MODE2 7
DRV8825 stepper(MOTOR_STEPS, DIR, STEP, MODE0, MODE1, MODE2);

#define DIR2 14
#define STEP2 15
#define MODE02 18
#define MODE12 17
#define MODE22 16
DRV8825 stepper2(MOTOR_STEPS, DIR2, STEP2, MODE02, MODE12, MODE22);

Servo esc1;
Servo esc2;
Servo esc3;
Servo servo1, servo2, servo3;

void joydata(const sensor_msgs::Joy& joy){
  int joy_butt = joy.buttons[2]; // Trigger to move servos 1 and 2
  int joy_yellow = joy.buttons[1]; // yellow button to move servos 1 and 2
  int joy_trig = joy.buttons[0];
  float joy_fb = joy.axes[1]; // forward and backward
  float joy_lr = joy.axes[0]; // left and right
  float joy_servo3 = joy.axes[4]; // small black joystick. Up is +1
  
  if (joy_butt == 1){
       new_pos1 = pos1 + (pos1*joy_fb);
       if (new_pos1 > 2000) {
        new_pos1= 2000;
       }
       
       new_pos2 = pos2 + (pos2*joy_lr);
       if (new_pos2 < 1000){
        new_pos2= 1000;
       }
    
       //first_servo_msg.data = new_pos1;
       //second_servo_msg.data = new_pos2;

       //servo_pos1.publish( &first_servo_msg);
       //servo_pos2.publish( &second_servo_msg);

       old_pos1 = new_pos1;
       old_pos2 = new_pos2;
       old_pos3 = new_pos3;
  }

  else if(joy_servo3 > 0){
      new_pos3 = pos3 + (pos3*joy_fb);
      old_pos3 = new_pos3;
      
      new_pos3_neck = pos3_neck + (pos3_neck*joy_lr);
      servo3.write(new_pos3_neck);
      old_pos3_neck = new_pos3_neck;
  }
  
  else if(joy_yellow == 1){
      new_pos1_neck = pos1_neck + (pos1_neck*joy_fb);
      new_pos2_neck = pos2_neck + (pos2_neck*joy_lr);
      
      servo1.write(new_pos1_neck);
      servo2.write(new_pos2_neck);
      
      old_pos1_neck = new_pos1_neck;
      old_pos2_neck = new_pos2_neck;
  }
  
  else {
     esc1.writeMicroseconds(1500);
     esc2.writeMicroseconds(1500);
     esc3.writeMicroseconds(1500);
     
     servo1.write(old_pos1_neck);
     servo2.write(old_pos2_neck);
     servo3.write(old_pos3_neck);

     //first_servo_msg.data = new_pos1;
     //second_servo_msg.data = new_pos2;

     //servo_pos1.publish( &first_servo_msg);
     //servo_pos2.publish( &second_servo_msg);
  }
esc1.writeMicroseconds(new_pos1);
esc2.writeMicroseconds(new_pos2);
esc3.writeMicroseconds(new_pos3);

if (joy_trig == 1){
   if (joy_fb > 0) {
      stepper.move(MOTOR_STEPS);
      delay(1);
    }
    else if (joy_fb < 0) {
      stepper.move(-MOTOR_STEPS);
    }
    else if (joy_lr > 0) {
      stepper2.move(MOTOR_STEPS);
      delay(1);
    }
    else if (joy_lr < 0) {
      stepper2.move(-MOTOR_STEPS);
    }
    else {
      stepper.move(0);
      stepper2.move(0);
    }
}

}


ros::Subscriber<sensor_msgs::Joy> sub_joy_button("left_joy", joydata);


void setup(){
  nh.initNode();
  nh.subscribe(sub_joy_button);
 // nh,advertise(esc_pos1);
  //nh.advertise(esc_pos2);
  //nh.advertise(esc_pos3);

  stepper.begin(RPM);
  stepper.enable();

// use only for tilt mechanism
  esc1.attach(2); // attach it to pin 10 for tilt
  esc2.attach(3); // attach it to pin 11 for tilt
  esc3.attach(4);
  esc1.writeMicroseconds(1500); // pause tilt
  esc2.writeMicroseconds(1500); // pause tilt
  esc3.writeMicroseconds(1500);
  
  servo1.attach(9, 500, 2500);
  servo2.attach(10, 500, 2500);
  servo3.attach(11, 500, 2500);
  servo1.write(pos1);
  servo2.write(pos2);
  servo3.write(pos3);

}

void loop() {
  nh.spinOnce();
}

