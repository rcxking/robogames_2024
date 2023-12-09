/*
 * robomagellan_2024.ino
 *
 * Arduino code to gather data from sensors and send motor commands for the
 * Robogames 2024 RoboMagellan competition.
 *
 * Bryant Pong
 * 12/3/23
 */

// Needed for I2C to GPS
#include <Wire.h>

// Sparkfun u-blox GNSS Library
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

#include <MicroNMEA.h>

// Buffer storing incoming GPS data
char nmeaBuffer[100];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));

// Cached GPS values
double latitude, longitude;

// Have cached GPS values been updated?
bool gps_values_found = false;

// GPS Manager
SFE_UBLOX_GNSS myGNSS;

/**
 * @brief Helper function to store the incoming GPS data.
 * @param incoming Next incoming character data from GPS.
 */
void SFE_UBLOX_GNSS::processNMEA(const char incoming) {
  nmea.process(incoming);
}

/**
 * @brief Checks for a GPS coordinate and publishes it through the Serial
 * interface.  If a coordinate cannot be found "INVALID" is returned.
 */
void ProcessGPSCommand() {
  if (gps_values_found) {
    Serial.println(String(latitude, 6) + " " + String(longitude, 6));
  } else {
    Serial.println("INVALID");
  }
}

/**
 * @brief Processes the next command.
 */
void ProcessCommand(const String& full_command) {
  Serial.print("Processing next command: ");
  Serial.println(full_command);

  // First word is the command, so find the first space
  const int first_space_idx = full_command.indexOf(' ');
  Serial.print("first_space_idx: ");
  Serial.println(first_space_idx);

  // If a space isn't found, use the full_command
  String command;
  if (first_space_idx != -1) {
    command = full_command.substring(0, first_space_idx);
  } else {
    command = full_command;
  }
  Serial.print("command: ");
  Serial.println(command);

  /*
   * List of supported commands:
   * 1) GPS - Returns a string of the current latitude/longitude
   */
   if (command == "GPS") {
     ProcessGPSCommand();
   } else {
    Serial.println("Unsupported");
   }
}

void setup() {
  // Wait for a connection to the Raspberry Pi
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Connection established to Raspberry Pi");

  // Establish GPS connection; on failure stay in setup()
  Wire.begin();
  if (myGNSS.begin() == false) {
    Serial.println(F("ERROR: u-blox GNSS module not detected at default I2C address"));
    while (1);
  }

  /*
   * GPS Configurations:
   */
  myGNSS.setI2COutput(COM_TYPE_UBX | COM_TYPE_NMEA);
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);
  myGNSS.setProcessNMEAMask(SFE_UBLOX_FILTER_NMEA_ALL);
  myGNSS.setProcessNMEAMask(SFE_UBLOX_FILTER_NMEA_GGA);
}

void loop() {
  // Wait for the next requested command
  if (Serial.available() > 0) {
    // Process the next command after removing any whitespace and newline
    String next_command = Serial.readString();
    next_command.trim();
    ProcessCommand(next_command);
  }

  // Update cached GPS values
  myGNSS.checkUblox();

  if (nmea.isValid()) {
    // New GPS data is available
    gps_values_found = true;

    // Convert the new coordinate to degrees and save them
    latitude = nmea.getLatitude() / 1000000.;
    longitude = nmea.getLongitude() / 1000000.;
    nmea.clear();
  }

  // Need a small delay to prevent Arduino thrashing
  delay(100);
}
