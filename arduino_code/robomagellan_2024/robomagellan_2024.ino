/*
 * robomagellan_2024.ino
 *
 * Arduino code to send velocity commands (rad/s) to the motor controllers and
 * reads/computes the wheel velocities in rad/s from the encoders.
 */

// Quadrature Encoders
#include <Encoder.h>

// Enable debug statements?
#define ENABLE_DEBUGS (0)

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
constexpr int LEFT_ENCODER_CHAN_A  = 3;
constexpr int LEFT_ENCODER_CHAN_B  = 2;
constexpr int RIGHT_ENCODER_CHAN_A = 19;
constexpr int RIGHT_ENCODER_CHAN_B = 18;

Encoder leftEncoder(LEFT_ENCODER_CHAN_A, LEFT_ENCODER_CHAN_B);
Encoder rightEncoder(RIGHT_ENCODER_CHAN_A, RIGHT_ENCODER_CHAN_B);

// Last time wheel angular velocities were calculated
unsigned long last_time_ms = 0;

// Time (in ms) to compute the wheel velocities
const unsigned long COMPUTE_TIME_MS = 100;

// Last wheel positions (4x ticks)
int32_t prev_left_ticks_4x  = 0;
int32_t prev_right_ticks_4x = 0;

/*
 * CimCoder V2 tick to radians conversion factor.
 *
 * The encoders are CimCoder V2 encoders which generate 20 ticks per
 * revolution.  These are attached to the motor shafts.  Also on the
 * motor shaft is an 11-tooth gear connected to a 72-tooth gear on
 * the drive shaft.
 *
 * 20 ticks = 2pi radians; 1 tick = pi/10 radians
 *
 *           pi           11-tooth gear   11*pi
 * 1 Tick =  -- radians * ------------- = ----- radians
 *           10           72-tooth gear    720
 */
const double TICKS_TO_RADIANS = (11.0 * PI / 720.0);

/*
 * Compute the difference between the current and previous tick count.
 * This handles tick rollover by temporarily promoting the tick counts
 * as int64_t before performing the difference.
 *
 * Parameters:
 *   cur_ticks_4x: Current ticks (4x counting)
 *   prev_ticks_4x: Previous ticks (4x counting)
 *
 * Returns:
 *   (int32_t): Difference of cur_ticks_4x - prev_ticks_4x
 */
int32_t CalculateTicksDifference(const int32_t cur_ticks_4x, const int32_t prev_ticks_4x) {
  const int64_t diff = cur_ticks_4x - prev_ticks_4x;
  return static_cast<int32_t>(diff);
}

void setup() {
  // Wait for a connection to the Raspberry Pi
  Serial.begin(115200);

  // Need a short 150 ms delay otherwise Serial.print() will execute twice
  delay(150);

  // Wait for Serial to become available
  while (!Serial) {
    delay(10);
  }

  // Reset encoder counts
  leftEncoder.write(0);
  rightEncoder.write(0);
}

void loop() {
  // Time to compute motor velocity?
  const unsigned long current_time_ms = millis();
  const unsigned long time_delta_ms   = current_time_ms - last_time_ms;

  if (time_delta_ms >= COMPUTE_TIME_MS) {
    /*
     * Encoder library uses 4x counting so need to divide read() ticks by 4 to
     * get the true tick count. The CimCoders should return 20 ticks per
     * revolution.
     */
    const int32_t cur_left_ticks_4x  = leftEncoder.read();
    const int32_t cur_right_ticks_4x = rightEncoder.read();

    // Compute 4x count tick deltas
    const int32_t delta_left_ticks_4x  = CalculateTicksDifference(cur_left_ticks_4x, prev_left_ticks_4x);
    const int32_t delta_right_ticks_4x = CalculateTicksDifference(cur_right_ticks_4x, prev_right_ticks_4x);

    // Compute the true number of ticks
    const float true_left_ticks        = cur_left_ticks_4x / 4.0;
    const float true_right_ticks       = cur_right_ticks_4x / 4.0;

    const float true_delta_left_ticks  = delta_left_ticks_4x / 4.0;
    const float true_delta_right_ticks = delta_right_ticks_4x / 4.0;

    const double time_delta_s   = time_delta_ms / 1000.0;

    // Compute wheel positions (rad)
    const double left_wheel_pos_rads  = true_left_ticks * TICKS_TO_RADIANS;
    const double right_wheel_pos_rads = true_right_ticks * TICKS_TO_RADIANS;

    // Compute wheel velocities (rad/s)
    const double left_wheel_vel_rad_per_sec  = (true_delta_left_ticks * TICKS_TO_RADIANS) / time_delta_s;
    const double right_wheel_vel_rad_per_sec = (true_delta_right_ticks * TICKS_TO_RADIANS) / time_delta_s;

    // Debugs for wheel velocities
#if ENABLE_DEBUGS
    Serial.print("cur_left_ticks_4x: ");
    Serial.print(cur_left_ticks_4x);
    Serial.print(" cur_right_ticks_4x: ");
    Serial.print(cur_right_ticks_4x);

    Serial.print(" true_left_ticks: ");
    Serial.print(true_left_ticks);
    Serial.print(" true_right_ticks: ");
    Serial.print(true_right_ticks);

    Serial.print(" left_wheel_pos_rads: ");
    Serial.print(left_wheel_pos_rads);
    Serial.print(" right_wheel_pos_rads: ");
    Serial.print(right_wheel_pos_rads);

    Serial.print(" left_wheel_vel_rad_per_sec: ");
    Serial.print(left_wheel_vel_rad_per_sec);
    Serial.print(" right_wheel_vel_rad_per_sec: ");
    Serial.println(right_wheel_vel_rad_per_sec);
#endif

    /*
     * Publish wheel velocities to the Raspberry Pi.  The data is a string of the format:
     *
     * L<left wheel position (rad)>,<left wheel velocity (rad/sec)>;R<right wheel position (rad)>,<right wheel velocity in rad/sec>;
     *
     * Serial.println() prints floating point values with 2 floating point
     * digits so we need to specify 8 sigfigs for more accuracy.
     */
    const String wheel_vel_str = "L"  + String(left_wheel_pos_rads, 8) +
                                 ","  + String(left_wheel_vel_rad_per_sec, 8) +
                                 ";R" + String(right_wheel_pos_rads, 8) +
                                 ","  + String(right_wheel_vel_rad_per_sec, 8) +
                                 ";";
    Serial.println(wheel_vel_str);

    // Update previous encoder counts
    prev_left_ticks_4x  = cur_left_ticks_4x;
    prev_right_ticks_4x = cur_right_ticks_4x;

    // Update last time wheel velocities were computed
    last_time_ms = current_time_ms;
  } // End time_delta_ms >= COMPUTE_TIME_MS
}
