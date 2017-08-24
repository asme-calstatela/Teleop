/*
  Ultrasonic Sensor HC-SR04 and Arduino Tutorial

  Crated by Dejan Nedelkovski,
  www.HowToMechatronics.com

  Modified by Francisco Moxo
  to work with ROS
  08-11-2017
*/
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>

ros::NodeHandle nh;

sensor_msgs::Range range_msg;

// Sensor Topics
ros::Publisher pub_range1( "/ultrasound1", &range_msg);
ros::Publisher pub_range2( "/ultrasound2", &range_msg);
ros::Publisher pub_range3( "/ultrasound3", &range_msg);
ros::Publisher pub_range4( "/ultrasound4", &range_msg);


char frameid[] = "/ultrasound";






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

  // Advertises Node
  nh.advertise(pub_range1);
  nh.advertise(pub_range2);
  nh.advertise(pub_range3);
  nh.advertise(pub_range4);
  
  range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg.header.frame_id =  frameid;
  range_msg.field_of_view = 0.523;  // fake
  range_msg.min_range = 0.020;
  range_msg.max_range = 4.00;
}

void loop() {

  // Prints the distance on the Serial Monitor
 
  // Sensor1
  range_msg.range = getRange_Ultrasound(trigPin1,echoPin1);
  range_msg.header.stamp = nh.now();
  pub_range1.publish( &range_msg);

  // Sensor1
  range_msg.range = getRange_Ultrasound(trigPin2,echoPin2);
  range_msg.header.stamp = nh.now();
  pub_range2.publish( &range_msg);


  // Sensor3
  range_msg.range = getRange_Ultrasound(trigPin3,echoPin3);
  range_msg.header.stamp = nh.now();
  pub_range3.publish( &range_msg);

  // Sensor4
  range_msg.range = getRange_Ultrasound(trigPin4,echoPin4);
  range_msg.header.stamp = nh.now();
  pub_range4.publish( &range_msg);


  
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
  return distance/100; //in meters
}

