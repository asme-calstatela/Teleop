/*
  Ultrasonic Sensor HC-SR04 and Arduino Tutorial

  Crated by Dejan Nedelkovski,
  www.HowToMechatronics.com

  Modified by Francisco Moxo
  to work with ROS
  added tf messages that correspond to particular sonar
  09-02-2017
*/
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
//Header for TF broadcast
#include <tf/transform_broadcaster.h>



char frameid[] = "base_link";


char base_link[] = "/base_link";



ros::NodeHandle nh;

sensor_msgs::Range range_msg1;
sensor_msgs::Range range_msg2;

geometry_msgs::TransformStamped t1;
geometry_msgs::TransformStamped t2;
tf::TransformBroadcaster broadcaster;

// Sensor Topics
ros::Publisher pub_range1( "/ultrasound1", &range_msg1);
ros::Publisher pub_range2( "/ultrasound2", &range_msg2);
//ros::Publisher pub_range3( "/ultrasound3", &range_msg);
//ros::Publisher pub_range4( "/ultrasound4", &range_msg);









// defines pins numbers
// Sensor 1
const int trigPin1 = 12;
const int echoPin1 = 13;

// Sensor 2
const int trigPin2 = 10;
const int echoPin2 = 11;

// Sensor 3
const int trigPin3 = 8;
const int echoPin3 = 9;

// Sensor 4
const int trigPin4 = 6;
const int echoPin4 = 7;

// defines variables
long duration;
float distance;

void setup() {
  //Sensor 1
  pinMode(trigPin1, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin1, INPUT); // Sets the echoPin as an Input

  //Sensor 2
  pinMode(trigPin2, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin2, INPUT); // Sets the echoPin as an Input

  //Sensor 3
  pinMode(trigPin3, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin3, INPUT); // Sets the echoPin as an Input

  //Sensor 4
  pinMode(trigPin4, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin4, INPUT); // Sets the echoPin as an Input



  nh.initNode();
  broadcaster.init(nh);

  // Advertises Node
  nh.advertise(pub_range1);
  nh.advertise(pub_range2);
  //  nh.advertise(pub_range3);
  //  nh.advertise(pub_range4);

  // Sensor 1
  range_msg1.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg1.header.frame_id =  "/ultrasound1";
  range_msg1.field_of_view = 0.523;  // fake
  range_msg1.min_range = 0.020;
  range_msg1.max_range = 4.00;

  t1.header.frame_id = base_link;
  t1.child_frame_id = "ultrasound1";
  t1.transform.translation.x = 2.0;
  t1.transform.translation.y = -1.0;
  t1.transform.rotation.x = 0.0;
  t1.transform.rotation.y = 0.0;
  t1.transform.rotation.z = -0.7071;
  t1.transform.rotation.w = 0.7071;
  



  // Sensor 2
  range_msg2.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg2.header.frame_id =  "/ultrasound2";
  range_msg2.field_of_view = 0.523;  // fake
  range_msg2.min_range = 0.020;
  range_msg2.max_range = 4.00;

  t2.header.frame_id = base_link;
  t2.child_frame_id = "/ultrasound2";
  t2.transform.translation.x = 2.0;
  t2.transform.translation.y = 1.0;
  t2.transform.rotation.x = 0.0;
  t2.transform.rotation.y = 0.0;
  t2.transform.rotation.z = 0.7071;
  t2.transform.rotation.w = 0.7071;



}

void loop() {

  // Sensor1

  range_msg1.range = getRange_Ultrasound(trigPin1, echoPin1);
  range_msg1.header.stamp = nh.now();
  t1.header.stamp = nh.now();
  broadcaster.sendTransform(t1);
  pub_range1.publish( &range_msg1);




  // Sensor2

  range_msg2.range = getRange_Ultrasound(trigPin2, echoPin2);
  range_msg2.header.stamp = nh.now();
  t2.header.stamp = nh.now();
  broadcaster.sendTransform(t2);
  pub_range2.publish( &range_msg2);


  //
  //  // Sensor3
  //  range_msg.range = getRange_Ultrasound(trigPin3, echoPin3);
  //  //  range_msg.header.frame_id =  "/ultrasound3";
  //  range_msg.header.stamp = nh.now();
  //  pub_range3.publish( &range_msg);
  //  t.header.frame_id = "/ultrasound3";
  //  t.child_frame_id = base_link;
  //  t.transform.translation.x = 1.0;
  //  t.transform.translation.y = -2.0;
  //  t.transform.rotation.x = 0.0;
  //  t.transform.rotation.y = 0.0;
  //  t.transform.rotation.z = 0.0;
  //  t.transform.rotation.w = 1.0;
  //  t.header.stamp = nh.now();
  //  broadcaster.sendTransform(t);



  //
  //  // Sensor4
  //  range_msg.range = getRange_Ultrasound(trigPin4, echoPin4);
  //  //  range_msg.header.frame_id =  "/ultrasound4";
  //  range_msg.header.stamp = nh.now();
  //  pub_range4.publish( &range_msg);



  nh.spinOnce();

}


float getRange_Ultrasound(int trigPin, int echoPin) {
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);

  // Calculating the distance
  distance = duration * 0.034 / 2;
  return distance/100 ; //in meters
}
