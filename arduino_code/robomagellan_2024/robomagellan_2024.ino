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

// Sparkfun u-blox GNSS V3 Library (SAM-M10Q GPS)
#include <SparkFun_u-blox_GNSS_v3.h>

// Sparkfun BN008x 9 DOF IMU Library
#include "SparkFun_BNO08x_Arduino_Library.h"

/*
 * When debugging the Arduino code it can be helpful to enable/disable certain
 * sensors.  For a competition run all these should be 1.
 */
#define ENABLE_GPS (1)
#define ENABLE_ENCODERS (1)
#define ENABLE_IMU (1)

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

// IMU manager (connected via I2C address 0x4B)
BNO08x myIMU;

/*
 * For the most reliable interaction with the SHTP bus, we need
 * to use hardware reset control and monitor the H_INT pin.  The
 * H_INT pin will go low when it's okay to talk on the SHTP bus.
 * Define as -1 to disable these features.
 */
#define BNO08X_INT (A4)
#define BNO08X_RST (A5)
#define BNO08X_ADDR (0x4B)

// To reduce traffic from the IMU only update the IMU readings periodically
unsigned long last_imu_time = 0;

/*
 * Sensor enum values.  These are (in order)):
 *
 * 1) GPS Latitude (degrees)
 * 2) GPS Longitude (degrees)
 * 3) GPS Altitude (meters)
 * 4) Left Wheel Encoder ticks
 * 5) Right Wheel Encoder ticks
 * 6-9) IMU Accelerometer (meters/second^2)
 * 11-13) IMU Gyroscope (radians/second)
 * 14-16) IMU Magnetometer (teslas)
 * 17-20) IMU Orientation Vector (quaternion)
 */
enum SensorValues {
  GPS_LATITUDE_DEGS = 0,
  GPS_LONGITUDE_DEGS,
  GPS_ALTITUDE_M,
  LEFT_ENCODER_TICKS,
  RIGHT_ENCODER_TICKS,
  ACC_X_MS2,
  ACC_Y_MS2,
  ACC_Z_MS2,
  GYR_X_RPS,
  GYR_Y_RPS,
  GYR_Z_RPS,
  MAG_X_T,
  MAG_Y_T,
  MAG_Z_T,
  QUAT_I,
  QUAT_J,
  QUAT_K,
  QUAT_REAL,
  NUM_SENSOR_VALUES
};

// Sensor values are cached here
float sensor_values[NUM_SENSOR_VALUES];

void setup() {
  // Wait for a connection to the Raspberry Pi
  Serial.begin(115200);

  // Need a short 150 ms delay otherwise Serial.print() will execute twice
  delay(150);

  // Wait for Serial to become available
  while (!Serial) {
    delay(10);
  }

  // Start I2C connection
  Wire.begin();

  // Set the I2C clock to high speed - 400kHz
  Wire.setClock(400000);

  // Establish GPS connection (if enabled); on failure stay in setup()
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

  // Initialize IMU (if enabled); on failure stay in setup()
#if ENABLE_IMU
  if (myIMU.begin(BNO08X_ADDR, Wire, BNO08X_INT, BNO08X_RST) == false) {
    Serial.println(F("IMU: Failed to initialize IMU"));
    while(1);
  }
  ConfigureIMU();
#endif
}

void loop() {
#if ENABLE_GPS
  // Update cached GPS values periodically to avoid spamming I2C bus
  if (millis() - last_gps_time > 1000) {
    // Update time GPS values were updated
    last_gps_time = millis();

    // Latitude and longitude need to be divided by 10^7 to get degrees
    sensor_values[GPS_LATITUDE_DEGS] = myGNSS.getLatitude() / 10000000.;
    sensor_values[GPS_LONGITUDE_DEGS] = myGNSS.getLongitude() / 10000000.;

    // Altitude needs to be divided by 1000 to get meters
    sensor_values[GPS_ALTITUDE_M] = myGNSS.getAltitudeMSL() * 1e-3;
  }
#endif

#if ENABLE_ENCODERS
  /*
   * Update encoder readings.  The library provides 4X counting, which means that
   * each real tick is 4 ticks.
   */
  sensor_values[LEFT_ENCODER_TICKS] = leftEncoder.read() / 4;
  sensor_values[RIGHT_ENCODER_TICKS] = rightEncoder.read() / 4;
#endif

#if ENABLE_IMU
  if (myIMU.wasReset()) {
    ConfigureIMU();
  }

  // Update IMU readings periodically to avoid spamming I2C bus
  if (millis() - last_imu_time > 30) {
    // Update time IMU values were updated
    last_imu_time = millis();

    // Update values from the IMU
    if (myIMU.getSensorEvent() == true) {
      const uint8_t sensor_event_id = myIMU.getSensorEventID();

      if (sensor_event_id == SENSOR_REPORTID_ACCELEROMETER) {
        // Accelerometer data received
        sensor_values[ACC_X_MS2] = myIMU.getAccelX();
        sensor_values[ACC_Y_MS2] = myIMU.getAccelY();
        sensor_values[ACC_Z_MS2] = myIMU.getAccelZ();
      } else if (sensor_event_id == SENSOR_REPORTID_GYROSCOPE_CALIBRATED) {
        // Gyro data received
        sensor_values[GYR_X_RPS] = myIMU.getGyroX();
        sensor_values[GYR_Y_RPS] = myIMU.getGyroY();
        sensor_values[GYR_Z_RPS] = myIMU.getGyroZ();
      } else if (sensor_event_id == SENSOR_REPORTID_MAGNETIC_FIELD) {
        // Magnetometer data received
        sensor_values[MAG_X_T] = myIMU.getMagX() * 1e-6;
        sensor_values[MAG_Y_T] = myIMU.getMagY() * 1e-6;
        sensor_values[MAG_Z_T] = myIMU.getMagZ() * 1e-6;
      } else if (sensor_event_id == SENSOR_REPORTID_ROTATION_VECTOR) {
        sensor_values[QUAT_I] = myIMU.getQuatI();
        sensor_values[QUAT_J] = myIMU.getQuatJ();
        sensor_values[QUAT_K] = myIMU.getQuatK();
        sensor_values[QUAT_REAL] = myIMU.getQuatReal();
      }
    }
  }
#endif

  /*
   * Publish the sensor data.  To reduce latency this is a single string of the
   * form:
   * S <sensor_values array>\r\n
   *
   * To help verify the data sent is accurate, look for the "S" character at the
   * beginning and the \r\n at the end.
   */
  String temp("S ");
  for (size_t i = 0; i < NUM_SENSOR_VALUES; ++i) {
    // Need special cases for the encoder ticks as int32_t values
    if ((i == LEFT_ENCODER_TICKS) || (i == RIGHT_ENCODER_TICKS)) {
      temp += String(int32_t(sensor_values[i]));
    } else {
      temp += String(sensor_values[i]);
    }
    temp += " ";
  }
  Serial.println(temp);

  // Need a small delay to prevent Arduino thrashing
  delay(20);
}

/*
 * Configure the IMU.  Per https://github.com/sparkfun/SparkFun_BNO08x_Arduino_Library/issues/2
 * the initial call to enabling any IMU sensor will always fail and cause a sensor reset.
 */
void ConfigureIMU() {
  // Enable accelerometer
  if (myIMU.enableAccelerometer() == false) {
    Serial.println(F("IMU: Failed to enable accelerometer"));
  }

  // Enable gyroscope
  if (myIMU.enableGyro() == false) {
    Serial.println(F("IMU: Failed to enable gyroscope"));
  }

  // Enable magnetometer
  if (myIMU.enableMagnetometer() == false) {
    Serial.println(F("IMU: Failed to enable magnetometer"));
  }

  // Enable rotation vector
  if (myIMU.enableRotationVector() == false) {
    Serial.println(F("IMU: Failed to enable rotation vector"));
  }

  // Small delay needed to allow configuration to take into effect
  delay(100);
}
