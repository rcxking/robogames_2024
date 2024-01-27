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
#define ENABLE_GPS (0)
#define ENABLE_ENCODERS (1)

// Cached GPS values
double latitude = 0.0;
double longitude = 0.0;

// Last known latitude/longitude
double last_latitude = 0.0;
double last_longitude = 0.0;

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

// Last known encoder ticks
int32_t last_left_encoder_ticks = -1;
int32_t last_right_encoder_ticks = -1;

void setup() {
  // Wait for a connection to the Raspberry Pi
  Serial.begin(115200);

  // Need a short 150 ms delay otherwise Serial.print() will execute twice
  delay(150);

  while (!Serial);
  // Establish GPS connection (if enabled); on failure stay in setup()
  Wire.begin();
#if ENABLE_GPS
  if (myGNSS.begin() == false) {
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
  /*
   * Update encoder readings.  The library provides 4X counting, which means that
   * each real tick is 4 ticks.
   */
  left_encoder_ticks = leftEncoder.read() / 4;
  right_encoder_ticks = rightEncoder.read() / 4;
#endif

  /*
   * Publish the sensor data.  To reduce latency this is a single string of the
   * form:
   * S <GPS latitude> <GPS longitude> <left encoder ticks> <right encoder ticks>\r\n
   *
   * To help verify the data sent is accurate, look for the "S" character at the
   * beginning and the \r\n at the end.
   *
   * Also only publish data if any of the sensor readings have changed to avoid
   * spamming.
   */
  if (!DoubleEquals(latitude, last_latitude) ||
      !DoubleEquals(longitude, last_longitude) ||
      (last_left_encoder_ticks != left_encoder_ticks) ||
      (last_right_encoder_ticks != right_encoder_ticks)) {
    Serial.println(String("S") + " " + String(latitude) + " " +
                   String(longitude) + " " + String(left_encoder_ticks) + " " +
                   String(right_encoder_ticks));
  }

  // Update the last readings:
#if ENABLE_GPS
  last_latitude = latitude;
  last_longitude = longitude;
#endif

#if ENABLE_ENCODERS
  last_left_encoder_ticks = left_encoder_ticks;
  last_right_encoder_ticks = right_encoder_ticks;
#endif

  // Need a small delay to prevent Arduino thrashing
  delay(20);
}

// Helper function to determine if two doubles equal each other
bool DoubleEquals(const double num1, const double num2) {
  return fabs(num1 - num2) <= 0.00001;
}
