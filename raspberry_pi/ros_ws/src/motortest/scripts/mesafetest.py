#!/usr/bin/env python3
"""
Optical sensor counter node -- runs on Raspberry Pi.

Counts state transitions on two MZ80 optical sensors (e.g., row markers)
connected to GPIO and publishes cumulative counts to ROS topics.

Each state change (HIGH->LOW or LOW->HIGH) increments the counter by 0.5,
so a full pass through a marker registers as 1.0.

ROS Topics Published:
    /sensor_counter_1 (std_msgs/Float32): Cumulative count for sensor 1
    /sensor_counter_2 (std_msgs/Float32): Cumulative count for sensor 2

Configuration (ROS parameters):
    ~sensor_pin_1 (int, default: 12): GPIO BCM pin for sensor 1
    ~sensor_pin_2 (int, default: 20): GPIO BCM pin for sensor 2
    ~rate_hz      (int, default: 10): Polling rate
"""

import time

import rospy
import RPi.GPIO as GPIO
from std_msgs.msg import Float32

COUNT_INCREMENT = 0.5   # Each edge event = half a pass


class SensorCounter:
    """Counts edge transitions on two optical sensors and publishes counts."""

    def __init__(self):
        self.sensor_pin_1 = rospy.get_param("~sensor_pin_1", 12)
        self.sensor_pin_2 = rospy.get_param("~sensor_pin_2", 20)
        rate_hz = rospy.get_param("~rate_hz", 10)

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.sensor_pin_1, GPIO.IN)
        GPIO.setup(self.sensor_pin_2, GPIO.IN)

        self.counter_1 = 0.0
        self.counter_2 = 0.0
        self.previous_state_1 = GPIO.input(self.sensor_pin_1)
        self.previous_state_2 = GPIO.input(self.sensor_pin_2)

        self.counter_1_pub = rospy.Publisher(
            "/sensor_counter_1", Float32, queue_size=10
        )
        self.counter_2_pub = rospy.Publisher(
            "/sensor_counter_2", Float32, queue_size=10
        )

        rospy.init_node("sensor_counter_node", anonymous=True)
        self._run(rospy.Rate(rate_hz))

    def _run(self, rate):
        while not rospy.is_shutdown():
            self._poll_sensor(1)
            self._poll_sensor(2)
            time.sleep(0.1)
            rate.sleep()

    def _poll_sensor(self, sensor_id):
        if sensor_id == 1:
            current = GPIO.input(self.sensor_pin_1)
            if current != self.previous_state_1:
                self.counter_1 += COUNT_INCREMENT
                rospy.loginfo(f"Sensor 1 edge detected -- count: {self.counter_1}")
                self.counter_1_pub.publish(Float32(self.counter_1))
            self.previous_state_1 = current
        else:
            current = GPIO.input(self.sensor_pin_2)
            if current != self.previous_state_2:
                self.counter_2 += COUNT_INCREMENT
                rospy.loginfo(f"Sensor 2 edge detected -- count: {self.counter_2}")
                self.counter_2_pub.publish(Float32(self.counter_2))
            self.previous_state_2 = current

    def cleanup(self):
        GPIO.cleanup()


if __name__ == "__main__":
    try:
        sensor_counter = SensorCounter()
    except rospy.ROSInterruptException:
        pass
    except Exception as exc:
        rospy.logerr(f"Sensor counter error: {exc}")
    finally:
        GPIO.cleanup()
