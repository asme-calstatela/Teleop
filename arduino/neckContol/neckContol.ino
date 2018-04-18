#include <PololuMaestro.h>
#ifdef SERIAL_PORT_HARDWARE_OPEN
#define maestroSerial SERIAL_PORT_HARDWARE_OPEN
#else
#include <SoftwareSerial.h>
SoftwareSerial maestroSerial(0, 1);
#endif
MicroMaestro maestro(maestroSerial);

// Pins for Servo
int xPin = A1;
int yPin = A0;
int x2Pin = A2;

// Initial Positi
int xPosition, yPosition, x2Position = 0;
int servo1 = 90, servo2 = 90, servo3 = 90; // Initilly all sevos at 90 degrees

// change in degrees
int delta = 5;

void setup() {
  // initialize serial communications at 9600 bps:
  //  Serial.begin(9600);
  maestroSerial.begin(9600);

  pinMode(xPin, INPUT);
  pinMode(yPin, INPUT);
  pinMode(x2Pin, INPUT);

  // Set Initla location of Servos to center
  maestro.setTarget(0, 6000);
  delay(5);
  maestro.setTarget(1, 6000);
  delay(5);
  maestro.setTarget(2, 6000);
  delay(5);

}

void loop() {
  xPosition = analogRead(xPin);
  //  x2Position = analogRead(x2Pin);

  if (xPosition > 600) {
    //Serial.print("X: ");
    //Serial.print(xPosition);
    if (servo1 < 179) {
      servo1 = servo1 + delta;
      //Serial.print(" X: ");
      //Serial.print(servo1);
      //Serial.print(" Servo: ");
      //      Serial.println(map(servo1, 0, 180, 4000, 8000));
      maestro.setTarget(0, map(servo1, 0, 180, 4000, 8000));

    }
  }
  else if (xPosition < 400) {
    //Serial.print("X: ");
    //Serial.print(xPosition);
    if (servo1 > 1 ) {
      servo1 = servo1 - delta;
      //Serial.print(" X: ");
      //Serial.print(servo1);
      //Serial.print(" Servo: ");
      //      Serial.println(map(servo1, 0, 190, 4000, 8000));
      maestro.setTarget(0, map(servo1, 0, 180, 4000, 8000));

    }
  }
  delay(5);
  yPosition = analogRead(yPin);
  if (yPosition > 600) {
    //Serial.print("Y: ");
    //Serial.print(Yosition);
    if (servo2 < 179) {
      servo2 = servo2 + delta;
      //Serial.print(" X: ");
      //Serial.print(servo2);
      //Serial.print(" Servo: ");
      maestro.setTarget(1, map(servo2, 0, 180, 4000, 8000));
      //Serial.println(map(servo2, 0, 190, 4000, 8000));
    }
  }
  else if (yPosition < 400) {
    //Serial.print("Y: ");
    //Serial.print(yPosition);
    if (servo2 > 1 ) {
      servo2 = servo2 - delta;
      //Serial.print(" X: ");
      //Serial.print(servo1);
      //Serial.print(" Servo: ");
      maestro.setTarget(1, map(servo2, 0, 180, 4000, 8000));
      //Serial.println(map(servo1, 0, 190, 4000, 8000));
    }
  }
  delay(5);
  x2Position = analogRead(x2Pin);

  if (x2Position > 600) {
    //Serial.print("X2: ");
    //Serial.print(x2Position);
    if (servo3 < 179) {
      servo3 = servo3 + delta;
      //Serial.print(" X: ");
      //Serial.print(servo1);
      //Serial.print(" Servo: ");
      maestro.setTarget(2, map(servo3, 0, 180, 4000, 8000));
      //Serial.println(map(servo1, 0, 190, 4000, 8000));
    }
  }
  else if (x2Position < 400) {
    //Serial.print("X: ");
    //Serial.print(xPosition);
    if (servo3 > 1 ) {
      servo3 = servo3 - delta;
      //Serial.print(" X: ");
      //Serial.print(servo1);
      //Serial.print(" Servo: ");
      maestro.setTarget(2, map(servo3, 0, 180, 4000, 8000));
      //Serial.println(map(servo1, 0, 190, 4000, 8000));
    }
  }


  delay(5); // add some delay between reads
}
