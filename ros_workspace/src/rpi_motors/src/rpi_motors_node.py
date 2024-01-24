#!/usr/bin/env

# rpi_motors_node.py
# Interface node from the Raspberry Pi to the Talon SRX motor controllers.
# !!! THIS SHOULD ONLY BE RUN ON THE RASPBERRY PI !!!
#
# Bryant Pong
# 1/18/24

from odometry.msg import Velocities
from rpi_motors.srv import RPIMotors
import rospy

# Current linear velocities (m/s)
cur_left_vel = 0.0
cur_right_vel = 0.0

# Callback to update current linear velocities
def HandleVelocityMsg(msg):
    cur_left_vel = msg.left_velocity
    cur_right_vel = msg.right_velocity
    rospy.loginfo('cur_left_vel: ' + str(cur_left_vel) + '; cur_right_vel: ' +
                  str(cur_right_vel))

# Callback to handle sending motor percentages
def HandleMotorCommand(req):
    rospy.loginfo('Received left motor: ' + str(req.left_desired_velocity) +
            'm/s; right motor:' + str(req.right_desired_velocity) + 'm/s')
    return True

def main():
    rospy.loginfo('Starting Raspberry Pi Motors Node')
    rospy.init_node('rpi_motors_node')
    
    # Start Subscriber to the current motor velocities
    rospy.Subscriber('/odometry/current_velocities', Velocities, HandleVelocityMsg)
    
    # Start service to send motor commands
    # TODO: Ensure pigpio daemon is started before sending commands
    motor_service = rospy.Service('rpi_motor_commands', RPIMotors, HandleMotorCommand)
    
    rospy.loginfo('Ready to process and send motor commands')
    
    # TODO: Make this a rosparam
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    main()
