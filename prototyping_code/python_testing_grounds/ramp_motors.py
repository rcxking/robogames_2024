#!/usr/bin/env python3

from rpi_motors.srv import RPIMotors, RPIMotorsRequest
import rospy
import time

def main():
    # Initialize ROS
    rospy.init_node('step_motors')

    # Wait for RPI motors service to be available
    rospy.wait_for_service('/rpi_motor_commands')

    motor_srv = rospy.ServiceProxy('/rpi_motor_commands', RPIMotors)

    motor_cmd = RPIMotorsRequest()
    #motor_cmd.left_desired_velocity = 0.0
    motor_cmd.right_desired_velocity = 0.0

    while not rospy.is_shutdown():
        # Ramp Up
        while motor_cmd.right_desired_velocity <= 1.0:
            rospy.loginfo('Ramping up right velocity: ' +
                    str(motor_cmd.right_desired_velocity))
            result = motor_srv(motor_cmd)
            time.sleep(10)
            motor_cmd.right_desired_velocity += 0.1
        while motor_cmd.right_desired_velocity >= -1.0:
            rospy.loginfo('Ramping down right velocity: ' +
                    str(motor_cmd.right_desired_velocity))
            result = motor_srv(motor_cmd)
            time.sleep(3)
            motor_cmd.right_desired_velocity -= 0.1

if __name__ == '__main__':
    main()
