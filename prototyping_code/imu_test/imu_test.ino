#include "ICM_20948.h"

#define SERIAL_PORT Serial
#define WIRE_PORT Wire

// Using I2C
ICM_20948_I2C myICM;

void setup() {
  SERIAL_PORT.begin(115200);

  // Wait for the serial port to be active
  while (!SERIAL_PORT) {
  }

  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
}

void loop() {
  // put your main code here, to run repeatedly:

}
