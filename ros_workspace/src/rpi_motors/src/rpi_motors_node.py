#!/usr/bin/env

# rpi_motors_node.py
# Interface node from the Raspberry Pi to the Talon SRX motor controllers.
# !!! THIS SHOULD ONLY BE RUN ON THE RASPBERRY PI !!!
#
# Bryant Pong
# 1/18/24

from rpi_motors.srv import RPIMotors
import rospy

# Callback to handle sending motor percentages
def HandleMotorCommand(req):
    rospy.loginfo('Received left motor: ' + str(req.left_desired_velocity) +
            'm/s; right motor:' + str(req.right_desired_velocity) + 'm/s')

    return True

def main():
    rospy.loginfo('Starting Raspberry Pi Motors Node')
    rospy.init_node('rpi_motors_node')
    s = rospy.Service('rpi_motor_commands', RPIMotors, HandleMotorCommand)
    rospy.loginfo('Ready to process and send motor commands')
    rospy.spin()

if __name__ == '__main__':
    main()
