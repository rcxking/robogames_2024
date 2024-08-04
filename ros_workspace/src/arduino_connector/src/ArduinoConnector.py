#!/usr/bin/env

# ArduinoConnector.py
#
# Node to connect to the Arduino.
#
# Bryant Pong
# 12/8/23

from arduino_connector.msg import SensorStates
from enum import Enum
from sensor_msgs.msg import Imu, MagneticField, NavSatFix
import rospy
import serial
import time

'''
Sensor value indices.  These MUST match the order found in the Arduino code.
'''
SENSOR_VALUE_NAMES = [
    'GPS_LATITUDE_DEGS',
    'GPS_LONGITUDE_DEGS',
    'GPS_ALTITUDE_M',
    'LEFT_ENCODER_TICKS',
    'RIGHT_ENCODER_TICKS',
    'ACC_X_MS2',
    'ACC_Y_MS2',
    'ACC_Z_MS2',
    'GYR_X_RPS',
    'GYR_Y_RPS',
    'GYR_Z_RPS',
    'MAG_X_T',
    'MAG_Y_T',
    'MAG_Z_T',
    'QUAT_I',
    'QUAT_J',
    'QUAT_K',
    'QUAT_REAL',
    'NUM_SENSOR_VALUES'
]

SensorValues = Enum('SensorValues', SENSOR_VALUE_NAMES)

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

        # IMU publishers
        self._imu_raw_pub = rospy.Publisher('/imu/raw', Imu, queue_size=10)
        self._imu_mag_pub = rospy.Publisher('/imu/mag', MagneticField, queue_size=10)

        # GPS Publisher
        self._gps_pub = rospy.Publisher('/gps', NavSatFix, queue_size=10)

        '''
        Publisher of all sensor data sent by the Arduino.  Currently includes:
        1) Current left/right encoder ticks
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
        Read next line from the Arduino.  Because the Arduino and PySerial code
        execute quickly, we need to ensure the data is complete before parsing.
        '''
        if self._ser.in_waiting > 0:
            # Read complete line from serial output (may be incomplete)
            line = self._ser.readline()

            # If no newline detected, continue to read
            while not '\\n' in str(line):
                # Keep reading in next chunk
                temp = self._ser.readline()

                # Longer strings may require multiple passes to get full line
                if temp.decode():
                    line = (line.decode() + temp.decode()).encode()

            # Decode from bytes
            line = line.decode()
            # Strip leading and trailing spaces
            line = line.strip()

            # Expecting our data to start with 'S'; if it doesn't it's a debug
            if line[0] != 'S':
                return
            else:
                rospy.loginfo(line)

            # Begin parsing data
            split_data = line.split(' ')

            # Latitude/Longitude
            latitude = float(split_data[SensorValues.GPS_LATITUDE_DEGS.value])
            longitude = float(split_data[SensorValues.GPS_LONGITUDE_DEGS.value])
            altitude = float(split_data[SensorValues.GPS_ALTITUDE_M.value])

            # Encoder ticks.  Positive means forward motion; negative
            # reverse motion.
            left_encoder_ticks = int(split_data[SensorValues.LEFT_ENCODER_TICKS.value])
            right_encoder_ticks = int(split_data[SensorValues.RIGHT_ENCODER_TICKS.value])

            # IMU fields.  These are:
            # Accelerometer XYZ (m/s^2)
            # Gyroscope XYZ (rad/s)
            # Magnetometer XYZ (Teslas)
            accel_x_ms2 = float(split_data[SensorValues.ACC_X_MS2.value])
            accel_y_ms2 = float(split_data[SensorValues.ACC_Y_MS2.value])
            accel_z_ms2 = float(split_data[SensorValues.ACC_Z_MS2.value])

            gyro_x_rps = float(split_data[SensorValues.GYR_X_RPS.value])
            gyro_y_rps = float(split_data[SensorValues.GYR_Y_RPS.value])
            gyro_z_rps = float(split_data[SensorValues.GYR_Z_RPS.value])

            mag_x_t = float(split_data[SensorValues.MAG_X_T.value])
            mag_y_t = float(split_data[SensorValues.MAG_Y_T.value])
            mag_z_t = float(split_data[SensorValues.MAG_Z_T.value])

            quat_i = float(split_data[SensorValues.QUAT_I.value])
            quat_j = float(split_data[SensorValues.QUAT_J.value])
            quat_k = float(split_data[SensorValues.QUAT_K.value])
            quat_real = float(split_data[SensorValues.QUAT_REAL.value])

            # Construct SensorStates message and publish
            curr_time = rospy.Time.now()
            sensor_msg = SensorStates()
            sensor_msg.stamp = curr_time

            if self._enable_encoders:
                sensor_msg.left_ticks = left_encoder_ticks
                sensor_msg.right_ticks = right_encoder_ticks

            if self._enable_gps:
                # Construct GPS message
                gps_msg = NavSatFix()
                gps_msg.header.stamp = curr_time

                gps_msg.latitude = latitude
                gps_msg.longitude = longitude
                gps_msg.altitude = altitude

                self._gps_pub.publish(gps_msg)

            if self._enable_imu:
                # Construct IMU messages
                imu_raw_msg = Imu()
                imu_raw_msg.header.stamp = curr_time
                imu_raw_msg.orientation_covariance[0] = -1
                imu_raw_msg.linear_acceleration_covariance[0] = -1
                imu_raw_msg.angular_velocity_covariance[0] = -1

                imu_mag_msg = MagneticField()
                imu_mag_msg.header.stamp = curr_time

                # Accelerometer
                imu_raw_msg.linear_acceleration.x = accel_x_ms2
                imu_raw_msg.linear_acceleration.y = accel_y_ms2
                imu_raw_msg.linear_acceleration.z = accel_z_ms2

                # Gyroscope
                imu_raw_msg.angular_velocity.x = gyro_x_rps
                imu_raw_msg.angular_velocity.y = gyro_y_rps
                imu_raw_msg.angular_velocity.z = gyro_z_rps

                # Orientation quaternion
                imu_raw_msg.orientation.w = quat_real
                imu_raw_msg.orientation.x = quat_i
                imu_raw_msg.orientation.y = quat_j
                imu_raw_msg.orientation.z = quat_k

                # Magnetometer (Teslas)
                imu_mag_msg.magnetic_field.x = mag_x_t
                imu_mag_msg.magnetic_field.y = mag_y_t
                imu_mag_msg.magnetic_field.z = mag_z_t

                self._imu_raw_pub.publish(imu_raw_msg)
                self._imu_mag_pub.publish(imu_mag_msg)

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

        # Flush the Serial input buffer
        self._ser.reset_input_buffer()

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
