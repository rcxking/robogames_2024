/*
 * robomagellan_2024.ino
 *
 * Arduino code to gather data from sensors and send motor commands for the
 * Robogames 2024 RoboMagellan competition.
 *
 * Bryant Pong
 * 12/3/23
 */

// Quadrature Encoders
#include <Encoder.h>

// Needed for I2C to GPS
#include <Wire.h>

// Sparkfun u-blox GNSS Library
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

/*
 * When debugging the Arduino code it can be helpful to enable/disable certain
 * sensors.  For a competition run all these should be 1.
 */
#define ENABLE_GPS (1)
#define ENABLE_ENCODERS (1)

// Cached GPS values
double latitude, longitude;

// Have cached GPS values been updated?
bool gps_values_found = false;

// To reduce I2C traffic from the GPS, only query the GPS every second
unsigned long last_gps_time = 0;

// GPS Manager
SFE_UBLOX_GNSS myGNSS;

/*
 * CIMCoder pins.  Each CIMCoder requires 2 pins for Channels A/B.  For best
 * performance these 2 pins should be attached to Arduino pins that support
 * interrupts.
 *
 * On an Arduino Mega, these pins are: 2, 3, 18-21.  However, pins 20 and 21 are
 * (respectively) SDA/SCL which are used for the I2C bus.
 *
 * Because the motors have a single gear attached, they are "reversed", so
 * Channel A should be the next pin from Channel B.
 */
constexpr int LEFT_ENCODER_CHAN_A = 3;
constexpr int LEFT_ENCODER_CHAN_B = 2;
constexpr int RIGHT_ENCODER_CHAN_A = 19;
constexpr int RIGHT_ENCODER_CHAN_B = 18;

Encoder leftEncoder(LEFT_ENCODER_CHAN_A, LEFT_ENCODER_CHAN_B);
Encoder rightEncoder(RIGHT_ENCODER_CHAN_A, RIGHT_ENCODER_CHAN_B);

/*
 * Number of ticks each encoder reads.  0 is the home position; positive ticks
 * indicate the motor is going forward; negative ticks indicates motor is
 * reversing.
 */
int32_t left_encoder_ticks = 0;
int32_t right_encoder_ticks = 0;

/**
 * @brief Checks for a GPS coordinate and publishes it through the Serial
 * interface.  If a coordinate cannot be found "INVALID" is returned.
 */
void ProcessGPSCommand() {
#if ENABLE_GPS
  if (gps_values_found) {
    Serial.println("RES " + String(latitude, 6) + " " + String(longitude, 6));
  } else {
    Serial.println("RES INVALID");
  }
#else
  Serial.println("RES INVALID");
#endif
}

/**
 * @brief Publishes the current encoder ticks as a string.  Format is:
 * "RES <left encoder ticks> <right encoder ticks>"
 */
void ProcessENCCommand() {
#if ENABLE_ENCODERS
  Serial.println("RES " + String(left_encoder_ticks) + " " + String(right_encoder_ticks));
#else
  Serial.println("RES INVALID");
#endif
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
   * 2) ENC - Returns a string of the current encoder ticks
   *
   * All commands' output should start with "RES " for easy processing on the
   * ROS side.
   */
  if (command == "GPS") {
    ProcessGPSCommand();
  } else if (command == "ENC") {
    ProcessENCCommand();
  } else {
    Serial.println("Unsupported");
  }
}

void setup() {
  // Wait for a connection to the Raspberry Pi
  Serial.begin(115200);

  // Need a short 150 ms delay otherwise Serial.print() will execute twice
  delay(150);

  while (!Serial);
  Serial.println("Connection established to Raspberry Pi");

  // Establish GPS connection (if enabled); on failure stay in setup()
  Wire.begin();
#if ENABLE_GPS
  if (myGNSS.begin() == false) {
    Serial.println(F("ERROR: u-blox GNSS module not detected at default I2C address"));
    while (1);
  }

  /*
   * GPS Configurations:
   * 1) Set the I2C port to output UBX only/turn off NMEA noise
   * 2) Save only the comm. port settings to flash and BBR
   */
  myGNSS.setI2COutput(COM_TYPE_UBX);
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);
#endif

  // Reset the encoders to home position
#if ENABLE_ENCODERS
  leftEncoder.write(0);
  rightEncoder.write(0);
#endif
}

void loop() {
  // Wait for the next requested command
  if (Serial.available() > 0) {
    // Process the next command after removing any whitespace and newline
    String next_command = Serial.readString();
    next_command.trim();
    ProcessCommand(next_command);
  }

#if ENABLE_GPS
  // Update cached GPS values periodically to avoid spamming I2C bus
  if (millis() - last_gps_time > 1000) {
    // Update time GPS values were updated
    last_gps_time = millis();

    gps_values_found = true;

    // Latitude and longitude need to be divided by 10^7 to get degrees
    latitude = myGNSS.getLatitude() / 10000000.;
    longitude = myGNSS.getLongitude() / 10000000.;
  }
#endif

#if ENABLE_ENCODERS
  // Update encoder readings
  left_encoder_ticks = leftEncoder.read();
  right_encoder_ticks = rightEncoder.read();
#endif

  // Need a small delay to prevent Arduino thrashing
  delay(100);
}
