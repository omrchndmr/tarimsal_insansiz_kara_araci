#!/usr/bin/env python3
"""
Arduino distance sensor data bridge node.

Reads float values from an Arduino over a serial connection (e.g. ultrasonic
distance in cm) and publishes them to the /arduino_data ROS topic.

ROS Topics Published:
    /arduino_data (std_msgs/Float32): Distance reading in cm

Configuration (ROS parameters):
    ~serial_port  (str, default: /dev/ttyACM0): Serial device path
    ~baud_rate    (int, default: 9600): Serial baud rate
"""

import rospy
import serial
from std_msgs.msg import Float32

SERIAL_PORT_DEFAULT = "/dev/ttyACM0"
BAUD_RATE_DEFAULT = 9600


def serial_to_ros():
    rospy.init_node("serial_to_ros", anonymous=True)

    port = rospy.get_param("~serial_port", SERIAL_PORT_DEFAULT)
    baud = rospy.get_param("~baud_rate", BAUD_RATE_DEFAULT)

    data_pub = rospy.Publisher("arduino_data", Float32, queue_size=10)

    ser = serial.Serial(port, baud, timeout=1)
    rospy.loginfo(f"Serial bridge started on {port} at {baud} baud.")

    while not rospy.is_shutdown():
        try:
            line = ser.readline().decode("utf-8").rstrip()
            if line:
                try:
                    float_data = float(line)
                    rospy.logdebug(f"Serial data: {float_data}")
                    data_pub.publish(float_data)
                except ValueError:
                    rospy.logwarn(f"Invalid serial data (not float): {line}")
        except rospy.ROSInterruptException:
            break
        except Exception as exc:
            rospy.logerr(f"Serial read error: {exc}")

    ser.close()


if __name__ == "__main__":
    try:
        serial_to_ros()
    except rospy.ROSInterruptException:
        pass
