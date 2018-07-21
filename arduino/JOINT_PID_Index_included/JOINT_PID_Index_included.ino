/********************************************************
  Joint PID
  Created on:5/4/2018 by Francisco Moxo
  Eddited on :5/4/2018 by Francisco Moxo
  This script zeros out the joint by finding the index of the 
  encoder. After zeroing out the joint the user can then insert a tick number and the 
  joint will navigate to it using a PID.

 Known issues: Zero is has a constant error from the actual location but is the same for all instances.
  
 ********************************************************/

#include <PID_v1.h>
#include <Encoder.h>
#include <Servo.h>
// Encoder Pins
#define encoder1  2
#define encoder2 3

int newPosition;
int zeroval;
int firstloop = 1;

//Potentiometer
#define angle   A0
// ESC Control Pin
#define escsig 7

// Index Pins
int analogInPin = A5;  // Analog input pin that the potentiometer is attached to
int sensorValue = 0;        // value read from the pot
int minval = 1023;  // Min value from the index
int index = 0; // 0 = off 1 = on

// Encoder Code Setup
Encoder myEnc(encoder1, encoder2);
// create servo object to control a servo
Servo myservo;
//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp = 20, Ki = 15, Kd = 5;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
int WindowSize = 2000;
unsigned long windowStartTime;
void setup()
{
  //initialize the variables we're linked to
  Setpoint = 100;
  myservo.attach(escsig);
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(20);
  myPID.SetOutputLimits(-300, 300);
  Serial.begin(9600);
  Serial.println("Control System on:");
  JointZero();



  delay(5000);

}
long oldPosition  = -999;


int EncRead() {
  newPosition = myEnc.read();
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
    //    Serial.println(newPosition);
    Input = newPosition;
  }


}
int JointZero() { // this function finds the lowest index value and sets it equal too zero
  Serial.print("Zero-ing joint!");
  myservo.write(90);
  delay(3000);
  //Find Zero in Positive direction
  EncRead();
  while (newPosition <= 400) {
    myservo.write(80); // speed cab be slowed or increased
    EncRead();
    Serial.print(Input);
    // read the analog in value:
    sensorValue = analogRead(analogInPin);
    Serial.print(" sensor = " );
    Serial.print(sensorValue);
    if (sensorValue < minval) {
      minval = sensorValue;
      zeroval = newPosition;
    }
    Serial.print(" min = " );
    Serial.println(minval);
    delay(5);
  }
  EncRead();
  myservo.write(90);
  Serial.println("Switching sides!");
  delay(2000);

  //Find Zero in new direction
  while (newPosition >= -400) {
    myservo.write(100);// speed cab be slowed or increased
    EncRead();
    Serial.print(Input);
    // read the analog in value:
    sensorValue = analogRead(analogInPin);
    Serial.print("sensor = " );
    Serial.print(sensorValue);
    if (sensorValue < minval) {
      minval = sensorValue;
      zeroval = newPosition;
    }
    Serial.print("min = " );
    Serial.println(minval);
    delay(5);

  }
  EncRead();
  myservo.write(90);
  Serial.print(" done finding Zero!");
  delay(3000);

}



int pos = 0;  // variable to store the servo position
int pos2 = 0;
int myout = 1500;
void loop()
{
  if (firstloop == 1) { // Move to Zero position and Zero out the new position
    Serial.println(" first loop");
    EncRead();
    Setpoint = zeroval;
    myPID.Compute();
    Serial.print(" "); Serial.print(Setpoint); Serial.print(","); Serial.println(Input); //Serial.print(" "); Serial.print(Output); Serial.print(" "); Serial.println(myout);
    myservo.writeMicroseconds(1500 - (int)Output);              // tell servo to go to position in variable 'pos'
    if (Setpoint == Input) {
        myservo.writeMicroseconds(1500);
      newPosition = 0;
      oldPosition = newPosition;
      Input = newPosition;
      firstloop = 0;
      Output =0;
      Serial.println("Joint Has  been zeroed out!");
      delay(3000);
    }

  }


//  else { // Alow input of user
//    EncRead();
//    // input variable
//    pos = analogRead(A0);
//    Setpoint = 0;//map(pos, 0, 1023, 0, 100); // This is the desired posion to go too
//    myPID.Compute();
//    Serial.print(" "); Serial.print(Setpoint); Serial.print(","); Serial.println(Input); //Serial.print(" "); Serial.print(Output); Serial.print(" "); Serial.println(myout);
//    myservo.writeMicroseconds(1500 - (int)Output);              // tell servo to go to position in variable 'pos'
//
//  }

}

