/*
 * pid_tuning.ino
 *
 * Helper program used to tune the PID constants for the Robomagellan robot.
 */

// Quadrature Encoders
#include <Encoder.h>

// PID motor control
#include "PID.h"

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

// PID motor controls and constants.  Use PWM pins for the controllers
double        KP                   = 1.125;
double        KI                   = 0.00001;
double        KD                   = 25.0;
constexpr int LEFT_CONTROLLER_PIN  = 10;
constexpr int RIGHT_CONTROLLER_PIN = 9;

PID leftPID(KP, KI, KD);
PID rightPID(KP, KI, KD);

void setup() {
  // Begin serial terminal connection
  Serial.begin(115200);

  // Need a short 150 ms delay otherwise Serial.print() will execute twice
  delay(150);

  // Wait for serial connection to be established
  while (!Serial) {
    delay(10);
  }

  Serial.println("Serial connection (115200 baud) established");

  // Reset encoder counts
  leftEncoder.write(0);
  rightEncoder.write(0);

  // Attach to motors
  leftPID.ConnectToMotor(LEFT_CONTROLLER_PIN);
  rightPID.ConnectToMotor(RIGHT_CONTROLLER_PIN);
}

void loop() {
  // Check for user input
  if (Serial.available() > 0) {
    /*
     * The user can send the following commands:
     * 1) L<velocity> - Command the left motor to spin at the desired velocity
     *                  (rad/s)
     * 2) R<velocity> - Command the right motor to spin at the desired velocity
     *                  (rad/s)
     * 3) P<Proportional Constant> - Change both motors' proportional constant
     *                               to the specified value
     * 4) I<Integral Constant> - Change both motors' integral constant to the
     *                           specified value
     * 5) D<Derivative Constant> - Change both motors' derivative constant to
     *                             the specified value
     */
    const String nextCommand = Serial.readStringUntil('\n');
    Serial.print("Received next command: ");
    Serial.println(nextCommand);

    /*
     * Ensure command is at least 2 characters long (1 character for the
     * specifier (L/R/P/I/D) and at least 1 character for the value).
     */
    if (nextCommand.length() >= 2) {
      const char directive = nextCommand[0];
      const double value = nextCommand.substring(1).toFloat();

      if (directive == 'L') {
        // Set desired left motor velocity
        Serial.print("Setting desired left motor rad/s: ");
        Serial.println(value, 9);
        leftPID.SetDesiredVelocity(value);
      } else if (directive == 'R') {
        // Set desired right motor velocity
        Serial.print("Setting desired right motor rad/s: ");
        Serial.println(value, 9);
        rightPID.SetDesiredVelocity(value);
      } else if (directive == 'P') {
        // Update proportional constants
        Serial.print("Updating KP to: ");
        Serial.println(value, 9);
        leftPID.SetKP(value);
        rightPID.SetKP(value);
        Serial.println("leftPID constants: " + leftPID.DisplayPIDConstants());
        Serial.println("rightPID constants: " + rightPID.DisplayPIDConstants());
      } else if (directive == 'I') {
        // Update integral constants
        Serial.print("Updating KI to: ");
        Serial.println(value, 9);
        leftPID.SetKI(value);
        rightPID.SetKI(value);
        Serial.println("leftPID constants: " + leftPID.DisplayPIDConstants());
        Serial.println("rightPID constants: " + rightPID.DisplayPIDConstants());
      } else if (directive == 'D') {
        // Update derivative constants
        Serial.print("Updating KD to: ");
        Serial.println(value, 9);
        leftPID.SetKD(value);
        rightPID.SetKD(value);
        Serial.println("leftPID constants: " + leftPID.DisplayPIDConstants());
        Serial.println("rightPID constants: " + rightPID.DisplayPIDConstants());
      } else {
        // Unknown directive
        Serial.println("ERROR: Unknown directive: " + nextCommand);
      }
    } else {
      Serial.print("ERROR: Ignoring invalid command: ");
      Serial.println(nextCommand);
    }
  }

  // Time to compute motor velocities?
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

    // Reset encoder counts for next cycle
    leftEncoder.write(0);
    rightEncoder.write(0);

    // Compute and send the next desired motor velocities
    leftPID.SendPWMCommand(left_wheel_vel_rad_per_sec);
    rightPID.SendPWMCommand(right_wheel_vel_rad_per_sec);

    // Update last time wheel velocities were computed
    last_time_ms = current_time_ms;
  }
}
