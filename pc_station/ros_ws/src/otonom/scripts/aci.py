#!/usr/bin/env python3
"""
Gyroscope data publisher node.

Reads BNO055 IMU orientation data from an Arduino over a serial port and
publishes yaw angle to the /gyro_data ROS topic.

ROS Topics Published:
    /gyro_data (std_msgs/Float32): Yaw angle in degrees

Configuration (ROS parameters):
    ~serial_port  (str, default: /dev/ttyUSB0): Serial device path
    ~baud_rate    (int, default: 9600): Serial baud rate
    ~rate_hz      (int, default: 20): Publishing frequency
"""

import rospy
import serial
from std_msgs.msg import Float32

SERIAL_PORT_DEFAULT = "/dev/ttyUSB0"
BAUD_RATE_DEFAULT = 9600
RATE_HZ_DEFAULT = 20


def read_serial():
    rospy.init_node("bno055_data_publisher", anonymous=True)

    port = rospy.get_param("~serial_port", SERIAL_PORT_DEFAULT)
    baud = rospy.get_param("~baud_rate", BAUD_RATE_DEFAULT)
    rate_hz = rospy.get_param("~rate_hz", RATE_HZ_DEFAULT)

    pub = rospy.Publisher("gyro_data", Float32, queue_size=10)
    rate = rospy.Rate(rate_hz)

    ser = serial.Serial(port, baud, timeout=1)
    rospy.loginfo(f"Opened serial port {port} at {baud} baud.")

    while not rospy.is_shutdown():
        try:
            if ser.in_waiting > 0:
                line = ser.readline().decode("utf-8", errors="ignore").strip()
                rospy.logdebug(f"Raw serial data: {line}")
                try:
                    data = float(line)
                    pub.publish(data)
                except ValueError:
                    rospy.logwarn(f"Cannot convert to float: {line}")
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down gyro publisher.")
            ser.close()
            break

        rate.sleep()


if __name__ == "__main__":
    try:
        read_serial()
    except rospy.ROSInterruptException:
        pass
