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

// Current left/right wheel velocities (rad/s)
double left_wheel_vel_rad_per_sec  = 0.0;
double right_wheel_vel_rad_per_sec = 0.0;

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
    // Encoder library uses 4x counting so need to divide read() ticks by 4 to get true tick count
    const int32_t true_left_ticks  = leftEncoder.read() / 4;
    const int32_t true_right_ticks = rightEncoder.read() / 4;

    // Compute/update wheel velocities
    const double time_delta_s = time_delta_ms / 1000.0;
    left_wheel_vel_rad_per_sec  = (true_left_ticks * TICKS_TO_RADIANS) / time_delta_s;
    right_wheel_vel_rad_per_sec = (true_right_ticks * TICKS_TO_RADIANS) / time_delta_s;

    // Debugs for wheel velocities
#if ENABLE_DEBUGS
    Serial.print("time_delta_s: ");
    Serial.print(time_delta_s);
    Serial.print(" TICKS_TO_RADIANS: ");
    Serial.print(TICKS_TO_RADIANS);
    Serial.print(" true_left_ticks: ");
    Serial.print(true_left_ticks);
    Serial.print(" true_right_ticks: ");
    Serial.print(true_right_ticks);
    Serial.print(" left_wheel_vel_rad_per_sec: ");
    Serial.print(left_wheel_vel_rad_per_sec);
    Serial.print(" right_wheel_vel_rad_per_sec: ");
    Serial.println(right_wheel_vel_rad_per_sec);
#endif

    /*
     * Publish wheel velocities to the Raspberry Pi.  The data is a string of the format:
     *
     * L<left wheel velocity in rad/sec>;R<right wheel velocity in rad/sec>;
     *
     * Serial.println() prints floating point values with 2 floating point
     * digits; this is adequate because 0.01 radians is about a half degree.
     */
    const String wheel_vel_str = "L" + String(left_wheel_vel_rad_per_sec, 8) + ";R" + String(right_wheel_vel_rad_per_sec, 8) + ";";
    Serial.println(wheel_vel_str);

    // Reset encoder counts for next cycle
    leftEncoder.write(0);
    rightEncoder.write(0);

    // Update last time wheel velocities were computed
    last_time_ms = current_time_ms;
  }
}
