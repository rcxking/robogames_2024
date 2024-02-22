#!/usr/bin/env python3

from rpi_motors.srv import RPIMotors, RPIMotorsRequest
import rospy
import time

def main():
    # Initialize ROS
    rospy.init_node('step_motors')

    # Wait for RPI motors service to be available
    rospy.wait_for_service('/rpi_motors/rpi_motor_commands')

    motor_srv = rospy.ServiceProxy('/rpi_motors/rpi_motor_commands', RPIMotors)

    motor_cmd = RPIMotorsRequest()
    motor_cmd.left_desired_velocity = 0.0
    motor_cmd.right_desired_velocity = 0.0

    while not rospy.is_shutdown():
        rospy.loginfo('1 m/s')
        motor_cmd.left_desired_velocity = 1.0
        motor_cmd.right_desired_velocity = 1.0
        result = motor_srv(motor_cmd)
        time.sleep(30)
        rospy.loginfo('5 m/s')
        motor_cmd.left_desired_velocity = 5.0
        motor_cmd.right_desired_velocity = 5.0
        result = motor_srv(motor_cmd)
        time.sleep(30)

if __name__ == '__main__':
    main()
