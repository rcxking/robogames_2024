#!/usr/bin/env

# ArduinoConnector.py
#
# Node to connect to the Arduino.
#
# Bryant Pong
# 12/8/23

from arduino_connector.msg import GPS

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

        # Current latitude and longitude (degrees)
        self._latitude = None
        self._longitude = None

        # Last GPS query time
        self._last_gps_query_time = rospy.Time.now()

        '''
        Publishers:
        1) GPS containing current latitude/longitude
        '''
        self._gps_pub = rospy.Publisher('current_gps_location', GPS, queue_size=10)

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
            self._ser = serial.Serial(port_name, 115200, timeout=1)

            # Give a 1 second delay for Arduino 150 ms to kick in
            time.sleep(1)

            rospy.loginfo('Successfully connected to Arduino on: ' + port_name)
            return True
        except IOError:
            rospy.loginfo('ERROR: Could not open port to Arduino on: ' + port_name)
            return False

    '''
    Helper function to read string data from the Arduino until a line with the
    specified phrase is read.

    search_term is a STRING indicating when to stop.

    Returns the first line containing search_term.
    '''
    def ReadLinesUntil(self, search_term):
        found_term = False

        while not found_term:
            '''
            readline() reads back a binary array.  Need to convert this into
            a proper string for parsing.
            '''

            # Strip trailing newline/carriage returns
            stripped_data = self._ser.readline().strip()

            # Convert binary array to ASCII string
            data = stripped_data.decode('ascii')

            rospy.loginfo('Arduino sent: ' + data)

            found_term = search_term in data

            if found_term:
                rospy.loginfo('Found ' + search_term + ' in string: ' + data)
                return data

    '''
    Helper function to query the GPS coordinates
    '''
    def QueryGPS(self):
        # To prevent spamming for GPS coordinates ask periodically
        # TODO: Make this a parameter.
        if rospy.Time.now() - self._last_gps_query_time >= rospy.Duration(1):
            rospy.loginfo('Querying current GPS location')
            self._ser.write('GPS'.encode())

            # Read Arduino lines until RES is found
            coordinate_str = self.ReadLinesUntil('RES ')

            # Ensure INVALID wasn't received
            if not 'INVALID' in coordinate_str:
                # Response string is of the form RES <latitude> <longitude>
                split_coordinate_str = coordinate_str.split(' ')

                self._latitude = float(split_coordinate_str[1])
                self._longitude = float(split_coordinate_str[2])

                rospy.loginfo('Latitude: ' + str(self._latitude) + '; Longitude: ' +
                      str(self._longitude))

            self._last_gps_query_time = rospy.Time.now()

        # Publish last known GPS coordinate
        gps_msg = GPS()
        gps_msg.stamp = rospy.Time.now()
        gps_msg.latitude = self._latitude
        gps_msg.longitude = self._longitude
        self._gps_pub.publish(gps_msg)

    '''
    Main loop.  These tasks will be executed sequentially until the node
    shuts down.
    '''
    def MainLoop(self):
        # Polling rate (Hz) TODO: Make this a parameter
        rate = rospy.Rate(10)

        rospy.loginfo('Starting Arduino Connector MainLoop()')

        while not rospy.is_shutdown():
            self.QueryGPS()
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
