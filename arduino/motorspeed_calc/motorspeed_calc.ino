/*
 * Caluclates the speed of the left and right wheel 
 * 
 * Created By: Jeovanny Reyes
 * Modified On: 6/12/18
 */
#include <Encoder.h>


Encoder Left_enc(2, 4); // Channel A to Pin 2. Channel B to Pin 4 
Encoder Right_enc(3, 5); // Channel A to Pin 3. Channel B to Pin 5
//   avoid using pins with LEDs attached

// Interrupt Pins attahced to Channel A
const byte interruptPin_left = 2; // Interrupt Pin for left encoder
const byte interruptPin_right = 3; // Interrupt Pin for right encoder

// Keeping Track of Encoder Ticks for both left and right
long positionLeft  = -999;
long positionRight = -999;

// Updates old encoder position
volatile long newLeft, newRight; // Originally "long" data type. Updates encoder ticks

void setup() {
  // Interrupts become active when Channel A goes from High to Low and vice versa
  attachInterrupt(digitalPinToInterrupt(interruptPin_left),do_enc_left,CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptPin_right),do_enc_right,CHANGE);
  
  Serial.begin(9600);
  Serial.println("Left and Right Encoder:");
 
}

void loop() {
  if (newLeft != positionLeft || newRight != positionRight) {
    Serial.print("Left = ");
    Serial.print(newLeft);
    Serial.print(", Right = ");
    Serial.print(newRight);
    Serial.println();
    //Serial.print("Left Speed = ");
    //Serial.print(); // Prints speed of left motor
    //Serial.print("Right Speed = ");
    //Serial.print(); // Prints speed of right motor
    //Serial.println();
    positionLeft = newLeft;
    positionRight = newRight;
  }
  
  // if a character is sent from the serial monitor,
  // reset both back to zero.
  if (Serial.available()) {
    Serial.read();
    Serial.println("Reset left and right count to zero");
    Left_enc.write(0);
    Right_enc.write(0);
  }
}

  void do_enc_left(){ // Interrupt Service Routine 
    newLeft = Left_enc.read(); // Takes about 1593/1606/1591 ticks for one revoution
  }

  void do_enc_right(){ // Interrupt Serivce Routine
    newRight = Right_enc.read(); // Takes about 1330
  }

  //float left_odom(){
  //}

  //float right_odom(){
  //}
  //}

