/*
 * Created By: Jeovanny Reyes
 * Created On: 4/15/18
 * 
 * This includes the interrupt service routine
 * 
 * Input: Channel A and Channel B (From both Left and Right DC Motors)
 * Output: Tick numbers for Left and Right DC Motor
 * 
 * Subscribers: None
 * Publishers: "/left_enc_ticks" for Left DC Motor and "/right_enc_ticks" for RIght DC Motor
 * 
 * Number of Ticks for One Revolution (360 degrees):
 *                                                    Left Motor: 1593/1606/1591
 *                                                    Right Motor:1330/2600
 * 
 * 
 * TO RUN:
 *        run 'roscore' on one terminal
 *        run 'rosrun rosserial_python serial_node.py /dev/ttyACMx' on another terminal
 *        Where 'x' is the port number your arduino is connected to
 */
 
#include <ros.h>
#include <Encoder.h>
#include <std_msgs/Float32.h> // Can be Integer data type as well

ros::NodeHandle nh;

std_msgs::Float32 left_enc_msg; // This instantiates the publisher string
std_msgs::Float32 right_enc_msg;

ros::Publisher left_enc_count("/left_enc_ticks", &left_enc_msg);
ros::Publisher right_enc_count("/right_enc_ticks", &right_enc_msg);

// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder left_enc(2,4); // Left Encoder
Encoder right_enc(3,5); // Right Encoder
//   avoid using pins with LEDs attached

void setup() {
  nh.initNode();
  nh.advertise(left_enc_count);
  nh.advertise(right_enc_count);
}

long oldPosition_left  = -999;
long oldPosition_right = -999;

void loop() {
  long newPosition_left = left_enc.read();
  long newPosition_right = right_enc.read();
  
  if (newPosition_left != oldPosition_left || newPosition_right != oldPosition_right) {
    
    left_enc_msg.data = newPosition_left;
    oldPosition_left = newPosition_left;
    
    right_enc_msg.data = newPosition_right;
    oldPosition_right = newPosition_right;
    
    left_enc_count.publish( &left_enc_msg);
    right_enc_count.publish( &right_enc_msg);
  }

 // if (newPosition_right != oldPosition_right) {
   // right_enc_msg.data = newPosition_right;
   // oldPosition_right = newPosition_right;
   // right_enc_count.publish( &right_enc_msg);
 // }
  
  nh.spinOnce();
  delay(1);
}
