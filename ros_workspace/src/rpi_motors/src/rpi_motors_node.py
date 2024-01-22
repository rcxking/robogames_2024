#!/usr/bin/env

# rpi_motors_node.py
# Interface node from the Raspberry Pi to the Talon SRX motor controllers.
# !!! THIS SHOULD ONLY BE RUN ON THE RASPBERRY PI !!!
#
# Bryant Pong
# 1/18/24

from rpi_motors.srv import RPIMotors
import rospy

# Last desired motor percentages (-100% - 100%)
last_left_motor_percentage = 0
last_right_motor_percentage = 0

# Callback to handle sending motor percentages
def HandleMotorCommand(req):
    rospy.loginfo('Received left motor: ' + str(req.left_power_percentage) +
            '%; right motor:' + str(req.right_power_percentage) + '%')
    # Ensure requested percentages are within range
    req_left_percent = max(-100, req.left_power_percentage)
    req_left_percent = min(100, req_left_percent)
    req_right_percent = max(-100, req.right_power_percentage)
    req_right_percent = min(100, req_right_percent)

    rospy.loginfo('Sending left motor: ' + str(req_left_percent) +
            '%; right motor: ' + str(req_right_percent) + '%')

    return True

def main():
    rospy.loginfo('Starting Raspberry Pi Motors Node')
    rospy.init_node('rpi_motors_node')
    s = rospy.Service('rpi_motor_commands', RPIMotors, HandleMotorCommand)
    rospy.loginfo('Ready to process and send motor commands')
    rospy.spin()

if __name__ == '__main__':
    main()
