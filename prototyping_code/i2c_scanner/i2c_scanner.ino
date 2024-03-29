/*
 * i2c_scanner - Utility to scan the addresses of all attached I2C devices.
 *
 * Borrowed from: https://playground.arduino.cc/Main/I2cScanner/
 */

 #include <Wire.h>

void setup() {
  Wire.begin();
  Serial.begin(115200);

  // Wait for Serial monitor
  while (!Serial);
  Serial.println("\nI2C Scanner ready");
}

void loop() {
  // Number of I2C devices found
  int num_devices = 0;

  for (byte address = 1; address < 127; ++address) {
    // Use the return value of endTransmission() to see if a device acknowledges the address
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();

    if (error == 0) {
      // Found an I2C device at this address
      Serial.print("I2C device found at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.print(address, HEX);
      Serial.println(" !");
      ++num_devices;
    } else if (error == 4) {
      // Unknown error
      Serial.print("Unknown error at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.println(address, HEX);
    }
  }

  if (num_devices == 0) {
    Serial.println("No I2C devices found\n");
  } else {
    Serial.println("Done scanning for I2C devices\n");
  }

  // Timeout before beginning next scan
  delay(5000);
}
