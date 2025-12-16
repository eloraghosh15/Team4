#include <Encoder.h>

// --- ENCODER PINS ---
// Right Encoder: Pin 13 & 16
const int encoderRightPinA = 13;
const int encoderRightPinB = 16;

// Left Encoder: Pin 15 & 14
const int encoderLeftPinA = 15; 
const int encoderLeftPinB = 14;

// Create Encoder Objects
Encoder encoderRight(encoderRightPinA, encoderRightPinB);
Encoder encoderLeft(encoderLeftPinA, encoderLeftPinB);

// Variables to store counts
long posLeft = 0;
long posRight = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("Encoder Test Ready");
  
  // OPTIONAL: Manually enable PCINT for Pin 13 support if the library needs help
  // Enabling PCINT0 (Port B) covers Pins 14, 15, 16.
  // Pin 13 (PC7) has NO hardware interrupt, so Encoder library MUST poll it.
  // We don't need manual PCINT code here because the library handles polling automatically
  // as long as we call .read() frequently.
}

void loop() {
  // Read current values
  long newLeft = encoderLeft.read();
  long newRight = encoderRight.read();
  
  // Only print if values change
  if (newLeft != posLeft || newRight != posRight) {
    posLeft = newLeft;
    posRight = newRight;
    
    Serial.print("Left: ");
    Serial.print(posLeft);
    Serial.print("\tRight: ");
    Serial.println(posRight);
  }
  
  // Small delay to keep Serial readable, but short enough to catch polling
  delay(10); 
}