/*
  Ultrasonic Sensor HC-SR04 and Arduino Tutorial

  Crated by Dejan Nedelkovski,
  www.HowToMechatronics.com

  Modified by Francisco Moxo
  to work with ROS
  09-05-2017
*/
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>

ros::NodeHandle nh;
//Front
sensor_msgs::Range range_msg1;
sensor_msgs::Range range_msg2;
sensor_msgs::Range range_msg3;
sensor_msgs::Range range_msg4;
//Back
sensor_msgs::Range range_msg5;
sensor_msgs::Range range_msg6;
sensor_msgs::Range range_msg7;
sensor_msgs::Range range_msg8;



// Sensor Topics Front
ros::Publisher pub_range1( "/USFL1", &range_msg1);
ros::Publisher pub_range2( "/USFL2", &range_msg2);
ros::Publisher pub_range3( "/USFR1", &range_msg3);
ros::Publisher pub_range4( "/USFR2", &range_msg4);

// Sensor Topics BAck
ros::Publisher pub_range5( "/USBL1", &range_msg5);
ros::Publisher pub_range6( "/USBL2", &range_msg6);
ros::Publisher pub_range7( "/USBR1", &range_msg7);
ros::Publisher pub_range8( "/USBR2", &range_msg8);









// defines pins numbers FRONT
// Sensor 1
const int trigPin1 = 30;
const int echoPin1 = 31;
// Sensor 2
const int trigPin2 = 32;
const int echoPin2 = 33;
// Sensor 3
const int trigPin3 = 34;
const int echoPin3 = 35;

// Sensor 4
const int trigPin4 = 36;
const int echoPin4 = 37;


// defines pins numbers BACK
// Sensor 5
const int trigPin5 = 38;
const int echoPin5 = 39;
// Sensor 6
const int trigPin6 = 40;
const int echoPin6 = 41;
// Sensor 7
const int trigPin7 = 42;
const int echoPin7 = 43;
// Sensor 8
const int trigPin8 = 44;
const int echoPin8 = 45;



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


  //Sensor 5
  pinMode(trigPin5, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin5, INPUT); // Sets the echoPin as an Input

  //Sensor 6
  pinMode(trigPin6, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin6, INPUT); // Sets the echoPin as an Input

  //Sensor 7
  pinMode(trigPin7, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin7, INPUT); // Sets the echoPin as an Input

  //Sensor 8
  pinMode(trigPin8, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin8, INPUT); // Sets the echoPin as an Input



  nh.initNode();


  // Advertises Node
  nh.advertise(pub_range1);
  nh.advertise(pub_range2);
  nh.advertise(pub_range3);
  nh.advertise(pub_range4);
  nh.advertise(pub_range5);
  nh.advertise(pub_range6);
  nh.advertise(pub_range7);
  nh.advertise(pub_range8);


  // Sensor 1
  range_msg1.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg1.header.frame_id =  "/USFL1";
  range_msg1.field_of_view = 0.523;
  range_msg1.min_range = 0.020;
  range_msg1.max_range = 4.00;

  // Sensor 2
  range_msg2.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg2.header.frame_id =  "/USFL2";
  range_msg2.field_of_view = 0.523;
  range_msg2.min_range = 0.020;
  range_msg2.max_range = 4.00;

  // Sensor 3
  range_msg3.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg3.header.frame_id =  "/USFR1";
  range_msg3.field_of_view = 0.523;
  range_msg3.min_range = 0.020;
  range_msg3.max_range = 4.00;

  // Sensor 4
  range_msg4.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg4.header.frame_id =  "/USFR2";
  range_msg4.field_of_view = 0.523;
  range_msg4.min_range = 0.020;
  range_msg4.max_range = 4.00;


  // Sensor 5
  range_msg5.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg5.header.frame_id =  "/USBL1";
  range_msg5.field_of_view = 0.523;
  range_msg5.min_range = 0.020;
  range_msg5.max_range = 4.00;

  // Sensor 6
  range_msg6.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg6.header.frame_id =  "/USBL2";
  range_msg6.field_of_view = 0.523;
  range_msg6.min_range = 0.020;
  range_msg6.max_range = 4.00;

  // Sensor 7
  range_msg7.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg7.header.frame_id =  "/USBR1";
  range_msg7.field_of_view = 0.523;
  range_msg7.min_range = 0.020;
  range_msg7.max_range = 4.00;

  // Sensor 8
  range_msg8.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg8.header.frame_id =  "/USBR2";
  range_msg8.field_of_view = 0.523;
  range_msg8.min_range = 0.020;
  range_msg8.max_range = 4.00;




}

void loop() {
  // Sensor1
  range_msg1.range = getRange_Ultrasound(trigPin1, echoPin1);
  range_msg1.header.stamp = nh.now();
  pub_range1.publish( &range_msg1);

  // Sensor2
  range_msg2.range = getRange_Ultrasound(trigPin2, echoPin2);
  range_msg2.header.stamp = nh.now();
  pub_range2.publish( &range_msg2);

  // Sensor3
  range_msg3.range = getRange_Ultrasound(trigPin3, echoPin3);
  range_msg3.header.stamp = nh.now();
  pub_range3.publish( &range_msg3);

  // Sensor4
  range_msg4.range = getRange_Ultrasound(trigPin4, echoPin4);
  range_msg4.header.stamp = nh.now();
  pub_range4.publish( &range_msg4);

  nh.spinOnce();

  // Sensor5
  range_msg5.range = getRange_Ultrasound(trigPin5, echoPin5);
  range_msg5.header.stamp = nh.now();
  pub_range5.publish( &range_msg5);

  // Sensor6
  range_msg6.range = getRange_Ultrasound(trigPin6, echoPin6);
  range_msg6.header.stamp = nh.now();
  pub_range6.publish( &range_msg6);

  // Sensor7
  range_msg7.range = getRange_Ultrasound(trigPin7, echoPin7);
  range_msg7.header.stamp = nh.now();
  pub_range7.publish( &range_msg7);

  // Sensor8
  range_msg8.range = getRange_Ultrasound(trigPin8, echoPin8);
  range_msg8.header.stamp = nh.now();
  pub_range8.publish( &range_msg8);


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
  return distance / 100 ; //in meters
}
