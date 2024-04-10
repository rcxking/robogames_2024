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

// Sparkfun ICM-20948 9 DOF IMU Library
#include <ICM_20948.h>

/*
 * When debugging the Arduino code it can be helpful to enable/disable certain
 * sensors.  For a competition run all these should be 1.
 */
#define ENABLE_GPS (1)
#define ENABLE_ENCODERS (1)
#define ENABLE_IMU (1)

// Cached GPS values
double latitude = 0.0;
double longitude = 0.0;

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

// IMU manager (connected via I2C address 0x69)
ICM_20948_I2C myICM;

// To reduce traffic from the IMU only update the IMU readings periodically
unsigned long last_imu_time = 0;

/*
 * Cached IMU values.  These are:
 * 1) Accelerometer XYZ (in milli-g's)
 * 2) Gyroscope XYZ (in degrees per second)
 * 3) Magnetometer XYZ (in micro teslas)
 */
float acc_x_mg = 0.0;
float acc_y_mg = 0.0;
float acc_z_mg = 0.0;

float gyr_x_dps = 0.0;
float gyr_y_dps = 0.0;
float gyr_z_dps = 0.0;

float mag_x_ut = 0.0;
float mag_y_ut = 0.0;
float mag_z_ut = 0.0;

void setup() {
  // Wait for a connection to the Raspberry Pi
  Serial.begin(115200);

  // Need a short 150 ms delay otherwise Serial.print() will execute twice
  delay(150);

  while (!Serial);
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
  bool imu_initialized = false;
  while (!imu_initialized) {
    myICM.begin(Wire, 1);

    Serial.print(F("Initialization of IMU returned: "));
    Serial.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok) {
      Serial.println("IMU: Retrying initialization");
      delay(500);
    } else {
      imu_initialized = true;
    }
  }
#endif
}

void loop() {
#if ENABLE_GPS
  // Update cached GPS values periodically to avoid spamming I2C bus
  if (millis() - last_gps_time > 1000) {
    // Update time GPS values were updated
    last_gps_time = millis();

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

#if ENABLE_IMU
  // Update IMU readings periodically to avoid spamming I2C bus
  if (millis() - last_imu_time > 30) {
    // Update time IMU values were updated
    last_imu_time = millis();

    // Update values from the IMU
    myICM.getAGMT();

    acc_x_mg = myICM.accX();
    acc_y_mg = myICM.accY();
    acc_z_mg = myICM.accZ();

    gyr_x_dps = myICM.gyrX();
    gyr_y_dps = myICM.gyrY();
    gyr_z_dps = myICM.gyrZ();

    mag_x_ut = myICM.magX();
    mag_y_ut = myICM.magY();
    mag_z_ut = myICM.magZ();
  }
#endif

  /*
   * Publish the sensor data.  To reduce latency this is a single string of the
   * form:
   * S <GPS latitude> <GPS longitude> <left encoder ticks> <right encoder ticks>
       <Accel X> <Accel Y> <Accel Z> <Gyro X> <Gyro Y> <Gyro Z> <Mag X> <Mag Y>
       <Mag Z>\r\n
   *
   * To help verify the data sent is accurate, look for the "S" character at the
   * beginning and the \r\n at the end.
   */
  Serial.println(String("S") + " " + String(latitude) + " " +
                 String(longitude) + " " + String(left_encoder_ticks) + " " +
                 String(right_encoder_ticks) + " " + String(acc_x_mg) + " " +
                 String(acc_y_mg) + " " + String(acc_z_mg) + " " +
                 String(gyr_x_dps) + " " + String(gyr_y_dps) + " " +
                 String(gyr_z_dps) + " " + String(mag_x_ut) + " " +
                 String(mag_y_ut) + " " + String(mag_z_ut));

  // Need a small delay to prevent Arduino thrashing
  delay(20);
}
