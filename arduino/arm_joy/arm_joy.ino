#include <Servo.h>
#include <ros.h>
#include <sensor_msgs/Joy.h>

ros::NodeHandle nh;

int maximumval= 1;
int minimumval= -1;

int pos1 = 1500;
int pos2 = 1500;
int pos3 = 1500;

int new_pos1 = 1500;
int new_pos2 = 1500;
int new_pos3 = 1500;

int old_pos1 = 1500;
int old_pos2 = 1500;
int old_pos3 = 1500;

Servo esc1;
Servo esc2;
Servo esc3;

void joydata(const sensor_msgs::Joy& joy){
  int joy_butt = joy.buttons[2]; // Trigger to move servos 1 and 2
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
  }
  
  else {
     esc1.writeMicroseconds(1500);
     esc2.writeMicroseconds(1500);
     esc3.writeMicroseconds(1500);

     //first_servo_msg.data = new_pos1;
     //second_servo_msg.data = new_pos2;

     //servo_pos1.publish( &first_servo_msg);
     //servo_pos2.publish( &second_servo_msg);
  }
esc1.writeMicroseconds(new_pos1);
esc2.writeMicroseconds(new_pos2);
esc3.writeMicroseconds(new_pos3);

}


ros::Subscriber<sensor_msgs::Joy> sub_joy_button("joy", joydata);


void setup(){
  nh.initNode();
  nh.subscribe(sub_joy_button);
 // nh,advertise(esc_pos1);
  //nh.advertise(esc_pos2);
  //nh.advertise(esc_pos3);
  

// use only for tilt mechanism
  esc1.attach(2); // attach it to pin 10 for tilt
  esc2.attach(3); // attach it to pin 11 for tilt
  esc3.attach(4);
  esc1.writeMicroseconds(1500); // pause tilt
  esc2.writeMicroseconds(1500); // pause tilt
  esc3.writeMicroseconds(1500);

}

void loop() {
  nh.spinOnce();
}

