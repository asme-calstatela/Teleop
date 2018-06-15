/*
 * Created By: Jeovanny Reyes
 * Created On: 4/15/18
 * 
 * Modified On: 6/13/18
 * 
 * Input: Channel A and Channel B (From both Left and Right DC Motors)
 * Output: Tick numbers for Left and Right DC Motor
 * 
 * Subscribers: None
 * Publishers: "/left_enc_ticks" for Left DC Motor and "/right_enc_ticks" for RIght DC Motor
 * 
 * Number of Ticks for One Revolution (360 degrees):
 *                                                    Left Motor: 1593/1606/1591
 *                                                    Right Motor:1596/1615
 * 
 * 
 * TO RUN:
 *        run 'roscore' on one terminal
 *        run 'rosrun rosserial_python serial_node.py /dev/ttyACMx' on another terminal
 *        Where 'x' is the port number your arduino is connected to
 *        
 * NOTE: DO NOT CHANGE PIN ASSIGNMENTS FOR ENCODERS!!!!!!!!!!!!!!!!!!
 */
 
#include <ros.h>
#include <Encoder.h>
#include <std_msgs/Float32.h> // Can be Integer data type as well

ros::NodeHandle nh;

std_msgs::Float32 left_enc_msg; // This instantiates the publisher string
std_msgs::Float32 right_enc_msg;

ros::Publisher left_enc_count("/left_enc_ticks", &left_enc_msg);
ros::Publisher right_enc_count("/right_enc_ticks", &right_enc_msg);

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
  long newPosition_left = left_enc.read(); // + values for cw. - values for ccw
  long newPosition_right = -1 * right_enc.read(); // + values for cw. - values for ccw
  
  if (newPosition_left != oldPosition_left || newPosition_right != oldPosition_right) {
    
    left_enc_msg.data = newPosition_left;
    oldPosition_left = newPosition_left;
    
    right_enc_msg.data = newPosition_right;
    oldPosition_right = newPosition_right;
    
    left_enc_count.publish( &left_enc_msg);
    right_enc_count.publish( &right_enc_msg);
  }


  nh.spinOnce();
  delay(1);
}
