/*
 * talon_test.ino
 *
 * Prototyping code for the Arduino to communicate with a Talon SRX Motor
 * Controller.
 *
 * Bryant Pong
 * 1/15/24
 */
#include <Servo.h>

// Motor controllers require PWM 
Servo talon_left, talon_right;

/*
 * Motor power percentages (indicates how fast the motor will be moving); range
 * from -100 (reverse full power) to 100 (forward full power).
 */
int left_percent = 0;
int right_percent = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("Serial connected");

  // Need a PWM pin to commuicate with the motor controllers
  talon_left.attach(9);
  talon_right.attach(11);

  delay(1000);
  Serial.println("Type in two percentages:");
}

void loop() {
  // The Talon SRX takes a value from 1000 - 2000.  1500 indicates a stop.
  const int left_pwm_val = left_percent * 5 + 1500;
  const int right_pwm_val = right_percent * 5 + 1500;

  talon_left.writeMicroseconds(left_pwm_val);
  talon_right.writeMicroseconds(right_pwm_val);

  // Accept new input from the user
  if (Serial.available() > 1) {
    const String next_line = Serial.readString();

    // Split the left and right values
    const int space_idx = next_line.indexOf(" ");

    if (space_idx != -1) {
      // Found a space.  Determine the left and right motor percentages
      const String left_req_str = next_line.substring(0, space_idx);
      const String right_req_str = next_line.substring(space_idx+1);

      // Convert strings to integers
      const int left_req_int = left_req_str.toInt();
      const int right_req_int = right_req_str.toInt();

      // Handle left motor
      if (left_req_int == 0) {
        Serial.println("Stopping left motor");
        left_percent = 0;
      } else if (left_req_int >= -100 && left_req_int <= 100) {
        left_percent = left_req_int;
        Serial.print("Setting left percentage to: ");
        Serial.print(left_percent);
        Serial.println("%");
      } else {
        Serial.println("ERROR: Invalid left percentage!");
      }

      // Handle right motor
      if (right_req_int == 0) {
        Serial.println("Stopping right motor");
        right_percent = 0;
      } else if (right_req_int >= -100 && right_req_int <= 100) {
        right_percent = right_req_int;
        Serial.print("Setting right percentage to: ");
        Serial.print(right_percent);
        Serial.println("%");
      } else {
        Serial.println("ERROR: Invalid right percentage!");
      }
    }
  }
}
