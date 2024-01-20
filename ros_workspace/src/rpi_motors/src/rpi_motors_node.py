#!/usr/bin/env

# rpi_motors_node.py
# Interface node from the Raspberry Pi to the Talon SRX motor controllers.
# !!! THIS SHOULD ONLY BE RUN ON THE RASPBERRY PI !!!
#
# Bryant Pong
# 1/18/24

import rospy

def main():
    rospy.loginfo('Starting Raspberry Pi Motors Node')
    rospy.init_node('rpi_motors_node')
    rospy.loginfo('Ready to process and send motor commands')
    rospy.spin()

if __name__ == '__main__':
    main()
