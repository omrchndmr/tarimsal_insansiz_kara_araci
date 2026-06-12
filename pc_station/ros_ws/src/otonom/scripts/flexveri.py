#!/usr/bin/env python3
"""
Flex sensor data publisher node.

Reads an analog flex sensor value from Arduino pin A0 via Firmata (pymata4)
and publishes the reading to the /analog_sensor_value ROS topic.

ROS Topics Published:
    /analog_sensor_value (std_msgs/Float32): Raw analog reading (0–1023)

Thresholds:
    Values above 300 trigger a "Stop" log warning.
"""

import rospy
from pymata4 import pymata4
from std_msgs.msg import Float32

ANALOG_TOPIC = "analog_sensor_value"
ANALOG_PIN = 0
STOP_THRESHOLD = 300
PUBLISH_INTERVAL_S = 0.3


def read_analog_data(board, publisher):
    while not rospy.is_shutdown():
        result = board.analog_read(ANALOG_PIN)
        value = result[0]

        if value > STOP_THRESHOLD:
            rospy.logwarn(f"Flex sensor threshold exceeded: {value} > {STOP_THRESHOLD}. Stop signal.")
        else:
            rospy.logdebug(f"Flex sensor A{ANALOG_PIN}: {value}")

        publisher.publish(value)
        rospy.sleep(PUBLISH_INTERVAL_S)


if __name__ == "__main__":
    rospy.init_node("analog_sensor_node", anonymous=True)

    analog_publisher = rospy.Publisher(ANALOG_TOPIC, Float32, queue_size=10)

    board = pymata4.Pymata4()
    board.set_pin_mode_analog_input(ANALOG_PIN)

    try:
        read_analog_data(board, analog_publisher)
    except (KeyboardInterrupt, RuntimeError):
        rospy.loginfo("Flex sensor node terminated.")
    finally:
        board.shutdown()
