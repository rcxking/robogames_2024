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
from rpi_motors.srv import RPIMotors
import pigpio
import rospy

# Class to perform the proportional motor controls
class RPIMotorsControl:
    # Default Constructor.
    def __init__(self):
        # Proportional constants
        self._kp_left = rospy.get_param('~kp_left')
        self._kp_right = rospy.get_param('~kp_right')
        rospy.loginfo('kp_left: ' + str(self._kp_left) + '; kp_right: ' + str(self._kp_right))

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

        # GPIO interface.  GPIO 18 is for the left motor; GPIO 13 is for the
        # right motor.
        self._pi = pigpio.pi()
        self._LEFT_GPIO_PIN = 18
        self._RIGHT_GPIO_PIN = 13

        # Send the stop motors command (1500)
        self.StopMotors()

        # Start Subscriber to the current motor velocities
        rospy.Subscriber('/odometry/current_velocities', Velocities, self.HandleVelocityMsg)

        # Start service to send motor commands
        motor_service = rospy.Service('rpi_motor_commands', RPIMotors, self.HandleMotorCommand)

        # Publisher to publish desired motor velocities
        self._desired_vel_pub = rospy.Publisher('rpi_motor_desired_velocities', Velocities, queue_size=10)

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
                      'm/s; right motor:' + str(req.right_desired_velocity) + 'm/s')
        self._des_left_vel = req.left_desired_velocity
        self._des_right_vel = req.right_desired_velocity
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

        # Apply proportional constants to determine the change in velocity (m/s)
        delta_left_vel_ms = self._kp_left * left_error
        delta_right_vel_ms = self._kp_right * right_error
        #rospy.loginfo('delta_left_vel_ms: ' + str(delta_left_vel_ms) + '; delta_right_vel_ms: ' + str(delta_right_vel_ms))

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
        rospy.loginfo('Changing kp_left to: ' + str(config.kp_left) + '; kp_right to: ' + str(config.kp_right))
        self._kp_left = config.kp_left
        self._kp_right = config.kp_right
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
