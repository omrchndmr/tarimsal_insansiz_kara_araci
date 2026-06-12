#!/usr/bin/env python3
"""
Hoe (çapa) mechanism controller node — runs on Raspberry Pi.

Controls the hoe lifting/lowering motors and soil probe actuators based on
plant detection signals and wheel encoder counters received over ROS.

Hardware:
    - Right/left hoe motors (RCAPA_PIN, LCAPA_PIN) via H-bridge
    - Right/left soil probe actuators (RPROB_PIN, LPROB_PIN)
    - Microswitch 1 (SWITCH_PIN): hoe lower limit
    - Microswitch 2 (SWITCH_PIN2): hoe upper limit

ROS Topics Subscribed:
    /detected_plant   (std_msgs/Float32): 1.0 = plant detected
    /object_positions_y (geometry_msgs/Point): Plant y-position in image
    /sensor_counter_1 (std_msgs/Float32): Encoder counter 1 (row length)
    /sensor_counter_2 (std_msgs/Float32): Encoder counter 2 (row length)
    /left_angle       (std_msgs/Float32): Current steering angle
    /ikisnaci         (std_msgs/Float32): Second-row entry flag

ROS Topics Published:
    /nerdeyim (std_msgs/Float32): Current operation zone (0=row, 2=turn, 3=row2, 4=done)
"""

import time

import rospy
import RPi.GPIO as GPIO
from geometry_msgs.msg import Point
from std_msgs.msg import Float32

# ---------------------------------------------------------------------------
# GPIO Pin Configuration
# ---------------------------------------------------------------------------
SWITCH_PIN = 17    # Microswitch 1 — hoe lower limit
SWITCH_PIN2 = 27   # Microswitch 2 — hoe upper limit
RCAPA_PIN = 4      # Right hoe motor
LCAPA_PIN = 13     # Left hoe motor
RPROB_PIN = 5      # Right probe actuator
LPROB_PIN = 6      # Left probe actuator

# ---------------------------------------------------------------------------
# Control Parameters
# ---------------------------------------------------------------------------
PWM_FREQUENCY = 1000       # Hz
PLANT_Y_THRESHOLD = 300    # Minimum image y-coordinate to trigger hoe action
ROW1_PLANT_COUNT_THRESHOLD = 1.9   # Encoder counts to start hoeing
ROW1_END_COUNT_THRESHOLD = 2.9     # Encoder counts to signal row end
ROW2_HOE_SPEED = 70        # PWM duty cycle for second-row hoe descent

# ---------------------------------------------------------------------------
# Module-level state (global callbacks share state via these variables)
# ---------------------------------------------------------------------------
bitkisayi = None
y_value = 0
capa_kontrol = 2
counter_1 = 0
counter_2 = 0
nerdeyim = 0
ikisnaci = 1
counter_reset_flag = False
timer_indir = 0
nerdeyim_pub = None
timer_kaldir = 0
son_counter1 = 0
son_counter2 = 0
analog_sensor_value = 0
flex_kontrol = 0
aci = 0
standartflex = 0

# ---------------------------------------------------------------------------
# GPIO Setup
# ---------------------------------------------------------------------------
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(SWITCH_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(SWITCH_PIN2, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(RCAPA_PIN, GPIO.OUT)
GPIO.setup(LCAPA_PIN, GPIO.OUT)
GPIO.setup(RPROB_PIN, GPIO.OUT)
GPIO.setup(LPROB_PIN, GPIO.OUT)

freq = PWM_FREQUENCY
rcapa = GPIO.PWM(RCAPA_PIN, freq)
lcapa = GPIO.PWM(LCAPA_PIN, freq)
rprob = GPIO.PWM(RPROB_PIN, freq)
lprob = GPIO.PWM(LPROB_PIN, freq)

rcapa.start(0)
lcapa.start(0)
rprob.start(0)
lprob.start(0)


# ---------------------------------------------------------------------------
# Helper Functions
# ---------------------------------------------------------------------------

def reset_counters():
    """Reset encoder counters once per activation cycle."""
    global counter_1, counter_2, counter_reset_flag
    if not counter_reset_flag:
        counter_1 = 0
        counter_2 = 0
        counter_reset_flag = True
        rospy.loginfo("Encoder counters reset.")
    else:
        rospy.logdebug("Counter reset already done — skipped.")


def capa_hareket(speed, direction):
    """
    Move the hoe mechanism.

    Args:
        speed (int): PWM duty cycle (0–100).
        direction (str): "indir" to lower, "kaldir" to raise.
    """
    if direction == "indir":
        rospy.loginfo("Lowering hoe.")
        rcapa.ChangeDutyCycle(0)
        lcapa.ChangeDutyCycle(speed)
        time.sleep(0.1)
    elif direction == "kaldir":
        rospy.loginfo("Raising hoe.")
        rcapa.ChangeDutyCycle(speed)
        lcapa.ChangeDutyCycle(0)
        time.sleep(0.1)


# ---------------------------------------------------------------------------
# ROS Callbacks
# ---------------------------------------------------------------------------

def ikisnaci_callback(msg):
    global ikisnaci
    ikisnaci = msg.data


def y_callback(msg):
    global y_value
    y_value = msg.y
    rospy.logdebug(f"Plant y-position: {y_value}")
    check_sensors()


def aci_callback(msg):
    global aci
    aci = msg.data
    check_sensors()


def object_position_callback(msg):
    global bitkisayi
    bitkisayi = msg.data


def counter_callback_1(data):
    global counter_1
    counter_1 = data.data
    rospy.logdebug(f"Counter 1: {counter_1}")
    check_sensors()


def counter_callback_2(data):
    global counter_2
    counter_2 = data.data
    rospy.logdebug(f"Counter 2: {counter_2}")
    check_sensors()


def check_sensors():
    """Main control logic — decides hoe position and operation zone transitions."""
    global son_counter1, son_counter2, counter_reset_flag, nerdeyim, capa_kontrol

    if nerdeyim_pub:
        nerdeyim_pub.publish(Float32(nerdeyim))

    if nerdeyim == 0:
        # Active row: lower hoe when plant is close and encoder count is sufficient
        if y_value > PLANT_Y_THRESHOLD and (counter_1 > ROW1_PLANT_COUNT_THRESHOLD
                                             or counter_2 > ROW1_PLANT_COUNT_THRESHOLD):
            rospy.loginfo("Lowering hoe — plant in range.")
            capa_hareket(100, "indir")
            son_counter1 = counter_1
            son_counter2 = counter_2

        if bitkisayi == 0:
            if counter_1 > ROW1_END_COUNT_THRESHOLD or counter_2 > ROW1_END_COUNT_THRESHOLD:
                nerdeyim = 2

    elif nerdeyim == 2:
        # Row transition: raise hoe before turning
        rospy.loginfo("Raising hoe for row transition.")
        capa_hareket(100, "kaldir")
        if ikisnaci > 0:
            nerdeyim = 3

    elif nerdeyim == 3:
        reset_counters()
        if y_value > PLANT_Y_THRESHOLD and (counter_1 > 1 or counter_2 > 1):
            rospy.loginfo("Lowering hoe on second row.")
            capa_hareket(ROW2_HOE_SPEED, "indir")
        if bitkisayi == 0:
            capa_hareket(80, "kaldir")
            time.sleep(6.0)
            nerdeyim = 4


# ---------------------------------------------------------------------------
# Entry Point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    try:
        rospy.init_node("capa_controller", anonymous=True)
        nerdeyim_pub = rospy.Publisher("/nerdeyim", Float32, queue_size=10)

        rospy.Subscriber("/detected_plant", Float32, object_position_callback)
        rospy.Subscriber("/object_positions_y", Point, y_callback)
        rospy.Subscriber("/sensor_counter_1", Float32, counter_callback_1)
        rospy.Subscriber("/sensor_counter_2", Float32, counter_callback_2)
        rospy.Subscriber("/left_angle", Float32, aci_callback, queue_size=10)
        rospy.Subscriber("/ikisnaci", Float32, ikisnaci_callback, queue_size=10)

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
    finally:
        rcapa.stop()
        lcapa.stop()
        rprob.stop()
        lprob.stop()
        GPIO.cleanup()
        rospy.loginfo("Hoe controller node terminated.")
