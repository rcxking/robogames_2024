/*
 * talon_srx_repl.ino
 *
 * REPL to send commands to the Talon SRX motor controllers connected.
 */

// Talon SRX motor controllers are connected via PWM pins
#include <Servo.h>

// SRX motor controller pins (should be PWM)
constexpr int LEFT_CONTROLLER_PIN  = 9;
constexpr int RIGHT_CONTROLLER_PIN = 10;

// Servo objects (1 for each controller)
Servo leftServo, rightServo;

/*
 * Talon SRX limits: The controllers take a PWM signal in the range [1000, 2000]
 * (microseconds).  A value of 1500 is STOP; 2000 is full-speed forward; 1000 is
 * full-speed reverse.
 */
constexpr int MAX_PWM  = 2000;
constexpr int MIN_PWM  = 1000;
constexpr int STOP_PWM = 1500;

/*
 * Helper function to send a desired PWM signal to a Talon SRX controller.
 * Also enforces the [1000, 2000] range.
 *
 * Parameters:
 *   pwm (int): Desired PWM to send (microseconds)
 *   servo (Servo *): Servo object to command
 */
void SendPWMCommand(const int pwm, const Servo *servo) {
  // nullptr checks
  if (servo == nullptr) {
    Serial.println("ERROR: Invalid servo passed!");
    return;
  }

  // If desired PWM is out-of-range send the STOP command instead
  int pwmToSend = ((pwm < MIN_PWM) || (pwm > MAX_PWM)) ? STOP_PWM : pwm;

  Serial.print("Desired PWM: ");
  Serial.print(pwm);
  Serial.print("; thresholded PWM: ");
  Serial.println(pwmToSend);

  servo->writeMicroseconds(pwmToSend);
}

void setup() {
  // Initialize Serial terminal connection
  Serial.begin(115200);

  // Need a short 150 ms delay otherwise Serial.print() will execute twice
  delay(150);

  // Wait for serial connection to be established
  while (!Serial) {
    delay(10);
  }

  Serial.println("Serial connection (115200 baud) established");

  // Connect to the Talon SRXs
  leftServo.attach(LEFT_CONTROLLER_PIN);
  rightServo.attach(RIGHT_CONTROLLER_PIN);
  Serial.println("Attached to Talon SRXs");

  // For safety send the STOP command to both controllers
  SendPWMCommand(STOP_PWM, &leftServo);
  SendPWMCommand(STOP_PWM, &rightServo);
}

void loop() {
  // Check for user input
  if (Serial.available() > 0) {
    /*
     * The user can send desired PWM commands of the form
     * L<PWM> or R<PWM>.
     */
    const String nextCommand = Serial.readStringUntil('\n');
    Serial.print("Received next command: ");
    Serial.println(nextCommand);

    // Ensure command is at least 2 characters long (1 character for the motor L/R, at least 1 character for PWM)
    if (nextCommand.length() >= 2) {
      const char motor = nextCommand[0];
      const int pwmCommand = nextCommand.substring(1).toInt();

      // Send motor command
      if (motor == 'L') {
        Serial.println("Writing to left motor");
        SendPWMCommand(pwmCommand, &leftServo);
      } else if (motor == 'R') {
        Serial.println("Writing to right motor");
        SendPWMCommand(pwmCommand, &rightServo);
      } else {
        // Unknown motor
        Serial.println("ERROR: Invalid motor specified");
      }
    } else {
      Serial.print("ERROR: Ignoring invalid command: ");
      Serial.println(nextCommand);
    }
  }
}
