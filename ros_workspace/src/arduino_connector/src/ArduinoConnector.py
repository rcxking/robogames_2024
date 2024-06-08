#!/usr/bin/env

# ArduinoConnector.py
#
# Node to connect to the Arduino.
#
# Bryant Pong
# 12/8/23

from arduino_connector.msg import SensorStates

import rospy
import serial
import time

'''
Helper class to establish and maintain a serial connection to the Arduino.
'''
class ArduinoConnector():
    # Constructor
    def __init__(self):
        # Serial connection to the Arduino
        self._ser = None

        # Enable/Disable sensors
        self._enable_gps = True
        self._enable_encoders = True
        self._enable_imu = True

        '''
        Publisher of all sensor data sent by the Arduino.  Currently includes:
        1) Current left/right encoder ticks
        2) GPS containing current latitude/longitude
        '''
        self._sensor_pub = rospy.Publisher(
                '/arduino_connector/current_sensor_states',
                SensorStates,
                queue_size=10)

    '''
    Establishes a connection to the Arduino.  The Arduino listens on the
    specified port with the following Serial settings:

    1) Baudrate: 115200
    2) Byte size: 8
    3) Parity: None
    4) Stop bits: 1

    port_name is a STRING.

    Returns True if the connection was established; False on error.
    '''
    def StartArduinoConnection(self, port_name):
        try:
            self._ser = serial.Serial(port_name, 115200, timeout=0)

            # Give a 1 second delay for Arduino 150 ms to kick in
            time.sleep(1)

            rospy.loginfo('Successfully connected to Arduino on: ' + port_name)
            return True
        except IOError:
            rospy.loginfo('ERROR: Could not open port to Arduino on: ' + port_name)
            return False

    '''
    Read and parse in the sensor data from Arduino.
    '''
    def ParseSensorData(self):
        '''
        Read the next sensor data line from the Arduino.  String format is:
        <GPS latitude> <GPS longitude> <left encoder ticks> <right encoder ticks>
        '''

        # Ensure there's data to read
        num_bytes_waiting = self._ser.in_waiting

        if num_bytes_waiting > 0:
            # Data is ready to be read.
            next_line_bytes = self._ser.read(num_bytes_waiting)

            # Convert string to ASCII
            next_line_str = next_line_bytes.decode()

            # Ensure the starting 'S' and traling '\n' are in the line:
            s_in_str = next_line_str[0] == 'S'
            newline_in_str = '\n' in next_line_str

            if s_in_str and newline_in_str:
                '''
                It's possible for multiple lines to be sent together.  Extract
                just the first line (up to the \r\n).
                '''
                first_newline_pos = next_line_str.index('\n')
                next_line_str = next_line_str[:first_newline_pos+1]

                # Strip the trailing newline/carriage return
                stripped_data = next_line_str.strip()

                split_data = stripped_data.split(' ')

                # Latitude/Longitude
                latitude = float(split_data[1])
                longitude = float(split_data[2])

                # Encoder ticks.  Positive means forward motion; negative
                # reverse motion.
                left_encoder_ticks = int(split_data[3])
                right_encoder_ticks = int(split_data[4])

                # IMU fields.  These are:
                # Accelerometer XYZ (m/s^2)
                # Gyroscope XYZ (rad/s)
                # Magnetometer XYZ (microteslas)
                accel_x_ms2 = float(split_data[5])
                accel_y_ms2 = float(split_data[6])
                accel_z_ms2 = float(split_data[7])

                gyro_x_rps = float(split_data[8])
                gyro_y_rps = float(split_data[9])
                gyro_z_rps = float(split_data[10])

                mag_x_ut = float(split_data[11])
                mag_y_ut = float(split_data[12])
                mag_z_ut = float(split_data[13])

                # Construct SensorStates message and publish
                sensor_msg = SensorStates()
                sensor_msg.stamp = rospy.Time.now()

                if self._enable_encoders:
                    sensor_msg.left_ticks = left_encoder_ticks
                    sensor_msg.right_ticks = right_encoder_ticks

                if self._enable_gps:
                    sensor_msg.latitude = latitude
                    sensor_msg.longitude = longitude

                if self._enable_imu:
                    # Accelerometer
                    sensor_msg.imu.linear_acceleration.x = accel_x_ms2
                    sensor_msg.imu.linear_acceleration.y = accel_y_ms2
                    sensor_msg.imu.linear_acceleration.z = accel_z_ms2

                    # Gyroscope
                    sensor_msg.imu.angular_velocity.x = gyro_x_rps
                    sensor_msg.imu.angular_velocity.y = gyro_y_rps
                    sensor_msg.imu.angular_velocity.z = gyro_z_rps

                    # Converting magnetometer readings to orientation.
                    # Because the robot is treated as a planar one, we only care
                    # about the heading given by the following piecewise
                    # equation relying on the magnetometer's x/y readings:
                    #
                    # Orientation is in DEGREES
                    # y > 0: 90 - atan(x/y)
                    # y < 0: 270 - atan(x/y)
                    # y = 0 && x < 0: 180
                    # y = 0 && x > 0: 0
                    rospy.loginfo('x: ' + str(mag_x_ut) + '; y: ' +
                            str(mag_y_ut) + '; z: ' + str(mag_z_ut))


                self._sensor_pub.publish(sensor_msg)

    '''
    Main loop.  These tasks will be executed sequentially until the node
    shuts down.
    '''
    def MainLoop(self):
        # Polling rate (Hz) TODO: Make this a parameter
        rate = rospy.Rate(200)

        rospy.loginfo('Starting Arduino Connector MainLoop()')

        # Enable/Disable features as needed
        self._enable_gps = rospy.get_param('~enable_gps')
        self._enable_encoders = rospy.get_param('~enable_encoders')
        self._enable_imu = rospy.get_param('~enable_imu')

        rospy.loginfo('enable_gps: ' + str(self._enable_gps) +
                      '; enable_encoders: ' + str(self._enable_encoders) +
                      '; enable_imu: ' + str(self._enable_imu))

        while not rospy.is_shutdown():
            self.ParseSensorData()
            rate.sleep()

if __name__ == '__main__':
    try:
        # Create ROS node
        rospy.init_node('arduino_connector', anonymous=True)

        connector = ArduinoConnector()

        # Start the connection
        connector.StartArduinoConnection('/dev/ttyACM0')

        # Begin main loop until shutdown requested
        connector.MainLoop()
    except rospy.ROSInterruptException:
        pass
