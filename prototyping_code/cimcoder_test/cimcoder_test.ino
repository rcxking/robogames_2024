#include <Encoder.h>

/*
 * Each CIMCoder requires 2 pins for Channel A/B.  For best performance these
 * 2 pins should be attached to Arduino pins that support interrupts.
 *
 * On an Arduino Mega, these pins are: 2, 3, 18-21
 * On an Arduino Uno, these pins are: 2, 3
 */
Encoder leftEncoder(19, 18);
Encoder rightEncoder(20, 21);

void setup() {
  Serial.begin(9600);
  Serial.println("CIMCoder Test");
}

long positionLeft = -999;
long positionRight = -999;

void loop() {
  long newLeft = leftEncoder.read();
  long newRight = rightEncoder.read();

  // Only publish updates if there are any changes
  if ((newLeft != positionLeft) || (newRight != positionRight)) {
    Serial.print("Left: ");
    Serial.print(newLeft);
    Serial.print("; Right: ");
    Serial.println(newRight);

    positionLeft = newLeft;
    positionRight = newRight;
  }
}
