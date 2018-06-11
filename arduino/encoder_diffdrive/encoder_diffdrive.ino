
#include <Encoder.h>

// Change these pin numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder Left_enc(2, 3);
Encoder Right_enc(4, 5); // Change to PIN 18 and 19 for interrupt capability
//   avoid using pins with LEDs attached

void setup() {
  Serial.begin(9600);
  Serial.println("Left and Right Encoder:");
}

long positionLeft  = -999;
long positionRight = -999;

void loop() {
  long newLeft, newRight;
  newLeft = Left_enc.read(); // Takes about 1593/1606/1591 ticks for one revoution
  newRight = Right_enc.read(); // Takes about 
  if (newLeft != positionLeft || newRight != positionRight) {
    Serial.print("Left = ");
    Serial.print(newLeft);
    Serial.print(", Right = ");
    Serial.print(newRight);
    Serial.println();
    positionLeft = newLeft;
    positionRight = newRight;
  }
  
  // if a character is sent from the serial monitor,
  // reset both back to zero.
  if (Serial.available()) {
    Serial.read();
    Serial.println("Reset left and right count to zero");
    //Left_enc.write(0);
    Right_enc.write(0);
  }
}
