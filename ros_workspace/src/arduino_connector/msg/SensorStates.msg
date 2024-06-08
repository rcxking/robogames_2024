# Current timestamp header
time stamp

# Left encoder ticks
int32 left_ticks

# Right encoder ticks
int32 right_ticks

# Current robot latitude (degrees)
float64 latitude

# Current robot longitude (degrees)
float64 longitude

# IMU fields:
# 1) Acceleration XYZ (meters/second^2)
# 2) Gyroscope XYZ (Radians/second)
# 3) Magnetometer XYZ (Microteslas)
sensor_msgs/Imu imu
