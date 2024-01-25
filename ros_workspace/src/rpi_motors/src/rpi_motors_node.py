#!/usr/bin/env

# rpi_motors_node.py
# Interface node from the Raspberry Pi to the Talon SRX motor controllers.
# !!! THIS SHOULD ONLY BE RUN ON THE RASPBERRY PI !!!
#
# Bryant Pong
# 1/18/24

from odometry.msg import Velocities
from rpi_motors.srv import RPIMotors
import pigpio
import rospy

# Proportional constants for the left and right sides
kp_left = 1.0
kp_right = 1.0

# Current linear velocities (m/s)
cur_left_vel = 0.0
cur_right_vel = 0.0

# Desired linear velocities (m/s)
des_left_vel = 0.0
des_right_vel = 0.0

# Current motor PWM signals (1000 - 2000); 1500 is STOP; 2000 is full forward;
# 1000 is full reverse.
cur_left_pwm = 1500
cur_right_pwm = 1500

# GPIO interface.  GPIO 18 is for the left motor; GPIO 13 is for the right.
pi = pigpio.pi()
LEFT_GPIO_PIN = 18
RIGHT_GPIO_PIN = 13

# Callback to update current linear velocities
def HandleVelocityMsg(msg):
    cur_left_vel = msg.left_velocity
    cur_right_vel = msg.right_velocity

# Callback to handle sending motor percentages
def HandleMotorCommand(req):
    rospy.loginfo('Received left motor: ' + str(req.left_desired_velocity) +
            'm/s; right motor:' + str(req.right_desired_velocity) + 'm/s')
    des_left_vel = req.left_desired_velocity
    des_right_vel = req.right_desired_velocity
    return True

# Compute next motor command
def ComputeMotorCommand():
    # Compute the error between the desired and current velocities (des - cur).
    # If the difference is positive, the current velocity is less than the
    # desired velocity and the motor needs to speed up.  If the difference
    # is negative, the current velocity is greater than the desired velocity and
    # the motor needs to slow up.  At a difference of 0 no additional changes
    # need to be made.
    left_error = des_left_vel - cur_left_vel
    right_error = des_right_vel - cur_right_vel
    
    # Apply proportional constants to determine the change in velocity (m/s)
    delta_left_vel_ms = kp_left * left_error
    delta_right_vel = kp_right * right_error
    
    # Convert velocities from m/s to motor commands
    delta_left_vel_pwm = 90.909 * delta_left_vel_ms + 1500
    delta_right_vel_pwm = 90.909 * delta_right_vel_ms + 1500
    
    # Apply current velocity commands
    cur_left_pwm += delta_left_vel_pwm
    cur_right_pwm += delta_right_vel_pwm
    
    # Threshold the current left/right pwm command to be in range [1000, 2000]
    cur_left_pwm = max(1000, cur_left_pwm)
    cur_left_pwm = min(2000, cur_left_pwm)
    
    cur_right_pwm = max(1000, cur_right_pwm)
    cur_right_pwm = min(2000, cur_right_pwm)
    
    # Send motor commands
    #pi.set_servo_pulsewidth(LEFT_GPIO_PIN, cur_left_pwm)
    #pi.set_servo_pulsewidth(RIGHT_GPIO_PIN, cur_right_pwm)

def main():
    rospy.loginfo('Starting Raspberry Pi Motors Node')
    rospy.init_node('rpi_motors_node')
    
    # Ensure pigpio daemon is started before sending commands
    if not pi.connected:
        rospy.loginfo('ERROR: pigpio daemon not started')
        return
    
    # Send the stop motors command (1500)
    pi.set_servo_pulsewidth(LEFT_GPIO_PIN, 1500)
    pi.set_servo_pulsewidth(RIGHT_GPIO_PIN, 1500)
    
    # Start Subscriber to the current motor velocities
    rospy.Subscriber('/odometry/current_velocities', Velocities, HandleVelocityMsg)
    
    # Start service to send motor commands
    # TODO: Ensure pigpio daemon is started before sending commands
    motor_service = rospy.Service('rpi_motor_commands', RPIMotors, HandleMotorCommand)
    
    rospy.loginfo('Ready to process and send motor commands')
    
    # TODO: Make this a rosparam
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        ComputeMotorCommand()
        rate.sleep()
        
    # On shutdown, stop the motors and perform cleanup
    rospy.loginfo('Stopping RPI motors node')
    pi.set_servo_pulsewidth(LEFT_GPIO_PIN, 1500)
    pi.set_servo_pulsewidth(RIGHT_GPIO_PIN, 1500)
    pi.stop()

if __name__ == '__main__':
    main()
