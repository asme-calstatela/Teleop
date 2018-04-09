/*
 *  Directions Usage:
 *    BOTTOM NECK SERVO:
 *                    CLOCKWISE: linear.z = -2
 *                    COUNTERCLOCKWISE: linear.z = 2
 *                    
 *    MIDDLE NECK SERVO:
 *                    CLOCKWISE: linear.z = -3
 *                    COUNTERCLOCKWISE  : linear.z = 3
 *                    
 *    MIDDLE NECK SERVO:
 *                    CLOCKWISE: linear.z = -4
 *                    COUNTERCLOCKWISE  : linear.z = 4
 */


#include <PololuMaestro.h>
#ifdef SERIAL_PORT_HARDWARE_OPEN
#define maestroSerial SERIAL_PORT_HARDWARE_OPEN
#else
#include <SoftwareSerial.h>
SoftwareSerial maestroSerial(0, 1);
#endif
MicroMaestro maestro(maestroSerial);

#include <Servo.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle nh;

// Pins for Servo
//int xPin = A1;
//int yPin = A0;
//int x2Pin = A2;

// Initial Position
//int xPosition, yPosition, x2Position = 0;
int servo1 = 90, servo2 = 90, servo3 = 90; // Initially all sevos at 90 degrees

// change in degrees
int delta = 5; // Change to increase how much servo rotates

void neck_cb(const geometry_msgs::Twist& cmd_msg){

  float linear_move = cmd_msg.linear.z // Information from key strokes

  //xPosition = analogRead(xPin);
  //  x2Position = analogRead(x2Pin);

  if linear_move == -2 {
  //if (xPosition > 600) {
    //Serial.print("X: ");
    //Serial.print(xPosition);
    //if (servo1 < 179) {
     servo1 = servo1 + delta;
      //Serial.print(" X: ");
      //Serial.print(servo1);
      //Serial.print(" Servo: ");
      //      Serial.println(map(servo1, 0, 180, 4000, 8000));
      //maestro.setTarget(0, map(servo1, 0, 180, 4000, 8000));
     maestro.setTarget(0, servo1);
      
    }
  //}
  else if linear_move == 2 {
  //else if (xPosition < 400) {
    //Serial.print("X: ");
    //Serial.print(xPosition);
    //if (servo1 > 1 ) {
     servo1 = servo1 - delta;
      //Serial.print(" X: ");
      //Serial.print(servo1);
      //Serial.print(" Servo: ");
      //      Serial.println(map(servo1, 0, 190, 4000, 8000));
      //maestro.setTarget(0, map(servo1, 0, 180, 4000, 8000));
      maestro.setTarget(0,servo1);
      
    }
  //}
  delay(5);

  if linear_move == -3 {
    servo2 = servo2 + delta;
    maestro.setTarget(1,servo2);
  }
  else if linear_move == 3 {
    servo2 = servo2 - delta;
    maestro.setTarget(1,servo2);
  }
  delay(5);

  if linear_move == -4 {
    servo3 = servo3 + delta;
    maestro.setTarget(2,servo3);
  }
  else if linear_move == 4 {
    servo3 = servo3 - delta;
    maestro.setTarget(2,servo3);
  }
  delay(5);
    
  //yPosition = analogRead(yPin);
  //if (yPosition > 600) {
    //Serial.print("Y: ");
    //Serial.print(Yosition);
    //if (servo2 < 179) {
      //servo2 = servo2 + delta;
      //Serial.print(" X: ");
      //Serial.print(servo2);
      //Serial.print(" Servo: ");
      //maestro.setTarget(1, map(servo2, 0, 180, 4000, 8000));
      //Serial.println(map(servo2, 0, 190, 4000, 8000));
    //}
  //}
  //else if (yPosition < 400) {
    //Serial.print("Y: ");
    //Serial.print(yPosition);
    //if (servo2 > 1 ) {
      //servo2 = servo2 - delta;
      //Serial.print(" X: ");
      //Serial.print(servo1);
      //Serial.print(" Servo: ");
      //maestro.setTarget(1, map(servo2, 0, 180, 4000, 8000));
      //Serial.println(map(servo1, 0, 190, 4000, 8000));
    //}
  //}
  delay(5);
  //x2Position = analogRead(x2Pin);

  //if (x2Position > 600) {
    //Serial.print("X2: ");
    //Serial.print(x2Position);
    //if (servo3 < 179) {
      //servo3 = servo3 + delta;
      //Serial.print(" X: ");
      //Serial.print(servo1);
      //Serial.print(" Servo: ");
      //maestro.setTarget(2, map(servo3, 0, 180, 4000, 8000));
      //Serial.println(map(servo1, 0, 190, 4000, 8000));
    //}
  //}
  //else if (x2Position < 400) {
    //Serial.print("X: ");
    //Serial.print(xPosition);
    //if (servo3 > 1 ) {
      //servo3 = servo3 - delta;
      //Serial.print(" X: ");
      //Serial.print(servo1);
      //Serial.print(" Servo: ");
      //maestro.setTarget(2, map(servo3, 0, 180, 4000, 8000));
      //Serial.println(map(servo1, 0, 190, 4000, 8000));
    //}
  //}


  //delay(5); // add some delay between reads
  
}

ros::Subscriber<geometry_msgs::Twist> sub_cmd_vel("cmd_vel", neck_cb)

void setup() {
  nh.initNode();
  nh.subscribe(sub_cmd_vel);
  // initialize serial communications at 9600 bps:
  //  Serial.begin(9600);
  //maestroSerial.begin(9600);

  //pinMode(xPin, INPUT);
  //pinMode(yPin, INPUT);
  pinMode(x2Pin, INPUT);

  // Set Initial location of Servos to center
  maestro.setTarget(0, 6000);
  delay(5);
  maestro.setTarget(1, 6000);
  delay(5);
  maestro.setTarget(2, 6000);
  delay(5);

}

void loop() {
  nh.spinOnce();
  delay(1);
}
