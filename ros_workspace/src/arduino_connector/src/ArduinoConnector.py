#!/usr/bin/env

# ArduinoConnector.py
#
# Node to connect to the Arduino.
#
# Bryant Pong
# 12/8/23

from arduino_connector.msg import GPS, Encoders

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

        # Current latitude and longitude (degrees)
        self._latitude = None
        self._longitude = None

        '''
        Current encoder ticks.  Positive ticks indicate that the motor is moving
        forward; negative ticks indicate the motor is moving backward.
        '''
        self._left_encoder_ticks = 0
        self._right_encoder_ticks = 0

        '''
        Publishers:
        1) GPS containing current latitude/longitude
        2) Current left/right encoder ticks
        '''
        self._gps_pub = rospy.Publisher(
                '/arduino_connector/current_gps_location',
                GPS,
                queue_size=10)
        self._encoders_pub = rospy.Publisher(
                '/arduino_connector/encoder_ticks',
                Encoders,
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

                self._latitude = float(split_data[1])
                self._longitude = float(split_data[2])
                self._left_encoder_ticks = int(split_data[3])
                self._right_encoder_ticks = int(split_data[4])

                # Publish last known GPS coordinate (if enabled)
                if self._enable_gps:
                    gps_msg = GPS()
                    gps_msg.stamp = rospy.Time.now()
                    gps_msg.latitude = self._latitude
                    gps_msg.longitude = self._longitude
                    self._gps_pub.publish(gps_msg)

                # Publish current encoder ticks (if enabled)
                if self._enable_encoders:
                    enc_msg = Encoders()
                    enc_msg.stamp = rospy.Time.now()
                    enc_msg.left_ticks = self._left_encoder_ticks
                    enc_msg.right_ticks = self._right_encoder_ticks
                    self._encoders_pub.publish(enc_msg)

    '''
    Main loop.  These tasks will be executed sequentially until the node
    shuts down.
    '''
    def MainLoop(self):
        # Polling rate (Hz) TODO: Make this a parameter
        rate = rospy.Rate(100)

        rospy.loginfo('Starting Arduino Connector MainLoop()')

        # Enable/Disable features as needed
        self._enable_gps = rospy.get_param('~enable_gps')
        self._enable_encoders = rospy.get_param('~enable_encoders')

        rospy.loginfo('enable_gps: ' + str(self._enable_gps) +
                      '; enable_encoders: ' + str(self._enable_encoders))

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
