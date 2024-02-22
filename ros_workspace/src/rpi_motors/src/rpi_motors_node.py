#!/usr/bin/env

# rpi_motors_node.py
# Interface node from the Raspberry Pi to the Talon SRX motor controllers.
# !!! THIS SHOULD ONLY BE RUN ON THE RASPBERRY PI !!!
#
# Bryant Pong
# 1/18/24

from dynamic_reconfigure.server import Server
from odometry.msg import Velocities
from rpi_motors.cfg import DynamicParamConfig
from rpi_motors.srv import RPIMotors, RPIMotorsLinAng
import pigpio
import rospy

# Class to perform the proportional motor controls
class RPIMotorsControl:
    # Default Constructor.
    def __init__(self):
        # Proportional constants
        self._kp_left = rospy.get_param('~kp_left')
        self._kp_right = rospy.get_param('~kp_right')
        self._ki_left = rospy.get_param('~ki_left')
        self._ki_right = rospy.get_param('~ki_right')
        self._kd_left = rospy.get_param('~kd_left')
        self._kd_right = rospy.get_param('~kd_right')
        rospy.loginfo('kp_left: ' + str(self._kp_left) +
                '; kp_right: ' + str(self._kp_right) +
                '; ki_left: ' + str(self._ki_left) +
                '; ki_right: ' + str(self._ki_right) +
                '; kd_left: ' + str(self._kd_left) +
                '; kd_right: ' + str(self._kd_right))

        # Current linear velocities (m/s)
        self._cur_left_vel = 0.0
        self._cur_right_vel = 0.0

        # Desired linear velocities (m/s)
        self._des_left_vel = 0.0
        self._des_right_vel = 0.0

        # Current motor PWM signals (1000 - 2000).  1500 is STOP; 2000 is full
        # forward; 1000 is full reverse.
        self._cur_left_pwm = 1500
        self._cur_right_pwm = 1500

        # Accumulated errors (for I term)
        self._accumulated_left_error = 0.0
        self._accumulated_right_error = 0.0

        # Previous errors (for D term)
        self._prev_left_error = 0.0
        self._prev_right_error = 0.0

        # GPIO interface.  GPIO 18 is for the left motor; GPIO 13 is for the
        # right motor.
        self._pi = pigpio.pi()
        self._LEFT_GPIO_PIN = 18
        self._RIGHT_GPIO_PIN = 13

        # Distance between the wheels (meters)
        self._ROBOT_WHEEL_DIST = 0.5715

        # Send the stop motors command (1500)
        self.StopMotors()

        # Start Subscriber to the current motor velocities
        rospy.Subscriber('/odometry/current_velocities', Velocities, self.HandleVelocityMsg)

        # Start service to send motor commands (left/right velocities)
        motor_service = rospy.Service('/rpi_motors/rpi_motor_commands', RPIMotors, self.HandleMotorCommand)

        # Service to send motor commands (linear/angular velocities)
        lin_ang_motor_service = rospy.Service('/rpi_motors/rpi_motor_commands_lin_ang', RPIMotorsLinAng, self.HandleLinAngMotorCommand)

        # Publisher to publish desired motor velocities
        self._desired_vel_pub = rospy.Publisher('/rpi_motors/rpi_motor_desired_velocities', Velocities, queue_size=10)

        # Setup dynamic reconfigure server
        self._reconfigure_server = Server(DynamicParamConfig, callback=self.DynamicReconfigureCallback)

        rospy.loginfo('Ready to process and send motor commands')

    # Callback to update robot's current linear velocities
    def HandleVelocityMsg(self, msg):
        self._cur_left_vel = msg.left_velocity
        self._cur_right_vel = msg.right_velocity

    # Callback to update receiving desired motor velocities
    def HandleMotorCommand(self, req):
        rospy.loginfo('Received left motor: ' + str(req.left_desired_velocity) +
                      ' m/s; right motor: ' + str(req.right_desired_velocity) + ' m/s')
        self._des_left_vel = req.left_desired_velocity
        self._des_right_vel = req.right_desired_velocity
        return True

    # Callback to update receiving desired linear/angular velocities
    def HandleLinAngMotorCommand(self, req):
        rospy.loginfo('Received linear: ' + str(req.linear_velocity) +
                      ' m/s; angular: ' + str(req.angular_velocity) + ' rad/s')

        # Convert linear/angular velocities to left and right side velocities
        self._des_right_vel = (self._ROBOT_WHEEL_DIST * req.angular_velocity / 2.0) + req.linear_velocity
        self._des_left_vel = (2 * req.linear_velocity) - self._des_right_vel

        rospy.loginfo('Setting desired left vel to: ' + str(self._des_left_vel) + ' m/s; right vel to: ' + str(self._des_right_vel) + ' m/s')
        return True

    # Helper to stop both motors
    def StopMotors(self):
        self._pi.set_servo_pulsewidth(self._LEFT_GPIO_PIN, 1500)
        self._pi.set_servo_pulsewidth(self._RIGHT_GPIO_PIN, 1500)

    # Compute the next motor commands
    def ComputeMotorCommand(self):
        # Compute the error between the desired and current velocities (des - cur).
        # If the difference is positive, the current velocity is less than the
        # desired velocity and the motor needs to speed up.  If the difference
        # is negative, the current velocity is greater than the desired velocity and
        # the motor needs to slow up.  At a difference of 0 no additional changes
        # need to be made.
        left_error = self._des_left_vel - self._cur_left_vel
        right_error = self._des_right_vel - self._cur_right_vel
        #rospy.loginfo('des_left_vel: ' + str(self._des_left_vel) + '; cur_left_vel: ' +
        #              str(self._cur_left_vel) + '; des_right_vel: ' + str(self._des_right_vel) +
        #              '; cur_right_vel: ' + str(self._cur_right_vel) + '; left_error: ' +
        #              str(left_error) + '; right_error: ' + str(right_error))

        # Add to accumulated errors integral term
        self._accumulated_left_error += (self._ki_left * left_error)
        self._accumulated_right_error += (self._ki_right * right_error)

        # Compute derivative term
        left_deriv = left_error - self._prev_left_error
        right_deriv = right_error - self._prev_right_error

        # Apply proportional constants to determine the change in velocity (m/s)
        delta_left_vel_ms = (self._kp_left * left_error) + (self._accumulated_left_error) + (self._kd_left * left_deriv)
        delta_right_vel_ms = (self._kp_right * right_error) + (self._accumulated_right_error) + (self._kd_right * right_deriv)
        #rospy.loginfo('delta_left_vel_ms: ' + str(delta_left_vel_ms) + '; delta_right_vel_ms: ' + str(delta_right_vel_ms))

        # Set the last error variables
        self._prev_left_error = left_error
        self._prev_right_error = right_error

        # Apply current velocity commands
        self._cur_left_pwm += delta_left_vel_ms
        self._cur_right_pwm += delta_right_vel_ms
        #rospy.loginfo('cur_left_pwm: ' + str(self._cur_left_pwm) + '; cur_right_pwm: ' + str(self._cur_right_pwm))

        # Threshold the current left/right pwm command to be in range [1000, 2000]
        self._cur_left_pwm = max(1000, self._cur_left_pwm)
        self._cur_left_pwm = min(2000, self._cur_left_pwm)

        self._cur_right_pwm = max(1000, self._cur_right_pwm)
        self._cur_right_pwm = min(2000, self._cur_right_pwm)
        #rospy.loginfo('Thresholded cur_left_pwm: ' + str(self._cur_left_pwm) + '; thresholded cur_right_pwm: ' + str(self._cur_right_pwm))

        # Send motor commands
        self._pi.set_servo_pulsewidth(self._LEFT_GPIO_PIN, self._cur_left_pwm)
        self._pi.set_servo_pulsewidth(self._RIGHT_GPIO_PIN, self._cur_right_pwm)

    # Callback to handle Dynamic Reconfigure parameters
    def DynamicReconfigureCallback(self, config, level):
        rospy.loginfo('Changing kp_left to: ' + str(config.kp_left) +
                '; kp_right to: ' + str(config.kp_right))
        self._kp_left = config.kp_left
        self._kp_right = config.kp_right

        rospy.loginfo('Changing ki_left to: ' + str(config.ki_left) +
                '; ki_right to: ' + str(config.ki_right))
        self._ki_left = config.ki_left
        self._ki_right = config.ki_right

        rospy.loginfo('Changing kd_left to: ' + str(config.kd_left) +
                '; kd_right to: ' + str(config.kd_right))
        self._kd_left = config.kd_left
        self._kd_right = config.kd_right
        return config

    # Main loop
    def spin(self):
        # TODO: Make this a rosparam
        rate = rospy.Rate(200)
        while not rospy.is_shutdown():
            self.ComputeMotorCommand()

            # Publish desired velocities
            des_vel_msg = Velocities()
            des_vel_msg.stamp = rospy.Time.now()
            des_vel_msg.left_velocity = self._des_left_vel
            des_vel_msg.right_velocity = self._des_right_vel
            des_vel_msg.linear_velocity = (self._des_left_vel + self._des_right_vel) / 2.0
            des_vel_msg.angular_velocity = (self._des_right_vel - self._des_left_vel) / self._ROBOT_WHEEL_DIST
            self._desired_vel_pub.publish(des_vel_msg)

            rate.sleep()

        # On shutdown, stop the motors and perform cleanup
        rospy.loginfo('Stopping RPI motors node')
        self.StopMotors()
        self._pi.stop()

def main():
    rospy.loginfo('Starting Raspberry Pi Motors Node')
    rospy.init_node('rpi_motors_node')

    # Construct motor controller object
    controller = RPIMotorsControl()
    controller.spin()

if __name__ == '__main__':
    main()
