#include <Wire.h>
#include "SparkFun_BNO08x_Arduino_Library.h"

BNO08x myIMU;

// Hardware reset control (can use any GPIO pins)
#define BNO08X_INT (A4)
#define BNO08X_RST (A5)

// Default I2C address
#define BNO08X_ADDR (0x4B)

void ConfigureIMU() {
  Serial.println("Configuring IMU");
  if (myIMU.enableAccelerometer() == true) {
    Serial.println(F("Accelerometer enabled"));
  } else {
    Serial.println("ERROR: Failed to initialize accelerometer");
  }

  // https://github.com/sparkfun/SparkFun_BNO08x_Arduino_Library/issues/2
  delay(100);
}

void setup() {
  Serial.begin(115200);

  // Wait for Serial to be available
  while (!Serial) {
    delay(10);
  }

  Serial.println();
  Serial.println("IMU Test");

  Wire.begin();

  // Initialize IMU
  if (myIMU.begin(BNO08X_ADDR, Wire, BNO08X_INT, BNO08X_RST) == false) {
    Serial.println("IMU not detected; check jumpers");
    while (1);
  }

  Serial.println("IMU found!");

  // Configure IMU
  ConfigureIMU();

  Serial.println("Ready to read from IMU");
  delay(100);
}

void loop() {
  if (myIMU.wasReset()) {
    Serial.println("IMU was reset");
    ConfigureIMU();
  }

  // Check if the IMU has new data
  if (myIMU.getSensorEvent() == true) {
    const uint8_t event_type = myIMU.getSensorEventID();
    if (event_type == SENSOR_REPORTID_ACCELEROMETER) {
      float x = myIMU.getAccelX();
      float y = myIMU.getAccelY();
      float z = myIMU.getAccelZ();

      Serial.print(x, 2);
      Serial.print(F(","));
      Serial.print(y, 2);
      Serial.print(F(","));
      Serial.print(z, 2);

      Serial.println();
    }
  }

  delay(10);
}
