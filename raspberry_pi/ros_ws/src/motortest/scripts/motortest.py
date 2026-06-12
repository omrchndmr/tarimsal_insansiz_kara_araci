#!/usr/bin/env python3
"""
Motor controller node — runs on Raspberry Pi.

Controls drive motors and steering via GPIO PWM signals based on plant
detection data received from the PC station over ROS.

Hardware:
    - 4 DC drive motors (two on each side) via H-bridge, BCM pins 18/19/7/8
    - 1 steering/rudder motor via H-bridge, BCM pins 16/26

ROS Topics Subscribed:
    /left_angle       (std_msgs/Float32): Steering angle from vision
    /detected_plant   (std_msgs/Float32): 1.0 = plant present, 0.0 = none
    /sensor_counter_1 (std_msgs/Float32): Wheel encoder counter 1
    /sensor_counter_2 (std_msgs/Float32): Wheel encoder counter 2
    /nerdeyim         (std_msgs/Float32): Current operation zone (0–4)
    /ikisnaci         (std_msgs/Float32): Second-row entry angle flag

ROS Topics Published:
    /ortalama (std_msgs/Float32): Averaging mode flag
"""

import time

import rospy
import RPi.GPIO as GPIO
from geometry_msgs.msg import Point
from std_msgs.msg import Float32

# ---------------------------------------------------------------------------
# GPIO Pin Configuration
# ---------------------------------------------------------------------------
RPWM_PIN = 18       # Right drive motor forward
LPWM_PIN = 19       # Right drive motor backward
RSAGON_PIN = 7      # Left drive motor forward
LSAGON_PIN = 8      # Left drive motor backward
RUDDER_RPWM = 16    # Steering motor right
RUDDER_LPWM = 26    # Steering motor left

# ---------------------------------------------------------------------------
# Control Parameters
# ---------------------------------------------------------------------------
PWM_FREQUENCY = 1000        # Hz
STEER_STRAIGHT_MIN = 88.5   # Angle range considered "straight ahead"
STEER_STRAIGHT_MAX = 91.5
PUBLISH_INTERVAL_S = 0.3    # Timer interval for autonomous state machine


class MotorController:
    """Controls drive and steering motors based on ROS angle commands."""

    def __init__(self):
        # State
        self.aci = 0            # Current steering angle
        self.acison = 3         # Last steering direction state
        self.counter_1 = 0
        self.counter_2 = 0
        self.nerdeyim = 0       # Current zone (0=row, 2=turning, 3=second row, 4=done)
        self.ikisnaci = 0       # Row-change angle flag
        self.bitkisayi = 0      # Detected plant count
        self.bisekine = 0       # Turn sub-state
        self.timer_donus = 0
        self.timer_bulma = 0
        self.timer_ileribul = 0
        self.duraktif = 0
        self.bitkiburda = 0     # Last known plant side (0=center, 1=right, 2=left)
        self.acigordu = 1       # Whether angle was seen in this cycle

        self._setup_gpio()

        # ROS
        self.ortalama_pub = rospy.Publisher("/ortalama", Float32, queue_size=10)
        self.left_angle_sub = rospy.Subscriber(
            "/left_angle", Float32, self.aci_callback, queue_size=10
        )
        self.bitkisayi_sub = rospy.Subscriber(
            "/detected_plant", Float32, self.bitkisayi_callback
        )
        self.counter_1_sub = rospy.Subscriber(
            "/sensor_counter_1", Float32, self.counter_callback_1
        )
        self.counter_2_sub = rospy.Subscriber(
            "/sensor_counter_2", Float32, self.counter_callback_2
        )
        self.nerdeyim_sub = rospy.Subscriber(
            "/nerdeyim", Float32, self.nerdeyim_callback
        )
        self.ikisnaci_sub = rospy.Subscriber(
            "/ikisnaci", Float32, self.ikisnaci_callback, queue_size=10
        )

        self.timer = rospy.Timer(
            rospy.Duration(PUBLISH_INTERVAL_S), self.expo
        )

    # ------------------------------------------------------------------
    # GPIO Setup
    # ------------------------------------------------------------------

    def _setup_gpio(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        for pin in [RPWM_PIN, LPWM_PIN, RSAGON_PIN, LSAGON_PIN, RUDDER_RPWM, RUDDER_LPWM]:
            GPIO.setup(pin, GPIO.OUT)

        freq = PWM_FREQUENCY
        self.rpwm = GPIO.PWM(RPWM_PIN, freq)
        self.lpwm = GPIO.PWM(LPWM_PIN, freq)
        self.rrsagon = GPIO.PWM(RSAGON_PIN, freq)
        self.llsagon = GPIO.PWM(LSAGON_PIN, freq)
        self.rpwmdumen = GPIO.PWM(RUDDER_RPWM, freq)
        self.lpwmdumen = GPIO.PWM(RUDDER_LPWM, freq)

        for pwm in [self.rpwm, self.lpwm, self.rrsagon, self.llsagon,
                    self.rpwmdumen, self.lpwmdumen]:
            pwm.start(0)

    # ------------------------------------------------------------------
    # ROS Callbacks
    # ------------------------------------------------------------------

    def aci_callback(self, msg):
        self.aci = msg.data
        self.man()

    def bitkisayi_callback(self, msg):
        self.bitkisayi = msg.data

    def counter_callback_1(self, data):
        self.counter_1 = data.data
        self.man()

    def counter_callback_2(self, data):
        self.counter_2 = data.data
        self.man()

    def nerdeyim_callback(self, msg):
        self.nerdeyim = msg.data
        self.man()

    def ikisnaci_callback(self, msg):
        self.ikisnaci = msg.data
        self.man()

    def x_callback(self, msg):
        self.x_value = msg.x
        self.man()

    # ------------------------------------------------------------------
    # State Machine
    # ------------------------------------------------------------------

    def expo(self, event):
        """Periodic timer callback used for initial straight-line driving."""
        if self.ikisnaci == 0:
            self.durDumen()
            self.move_forward(100)
            time.sleep(30)
            self.sag(100)
            time.sleep(7)
            self.durDumen()

    def man(self):
        """Main steering decision logic based on current angle and zone."""
        if self.nerdeyim in (0, 1):
            self._steer_in_row()
        elif self.nerdeyim == 2:
            self._row_transition()
        elif self.nerdeyim == 3:
            self._steer_second_row()

        if self.nerdeyim == 4:
            time.sleep(9.31)
            self.dur()

    def _steer_in_row(self):
        if STEER_STRAIGHT_MIN <= self.aci <= STEER_STRAIGHT_MAX:
            rospy.loginfo("Vehicle aligned — driving straight.")
            self.durDumen()
            self.move_forward(100)
            self.duraktif = 1
            self.bitkiburda = 0
            self.timer_bulma = 201
        elif 1 < self.aci < STEER_STRAIGHT_MIN:
            self.sag(100)
            self.move_forward(100)
            rospy.logdebug("Steering right.")
            self.duraktif = 1
            self.bitkiburda = 1
            self.timer_bulma = 201
        elif self.aci > STEER_STRAIGHT_MAX:
            self.sol(100)
            self.move_forward(100)
            rospy.logdebug("Steering left.")
            self.duraktif = 1
            self.bitkiburda = 2
            self.timer_bulma = 201
        else:
            rospy.logdebug("No valid angle — searching for plant.")
            if self.duraktif == 1:
                if self.bitkisayi == 0:
                    self.move_forward(70)
                    self.durDumen()
                else:
                    self.bitki_kaybol()

    def _steer_second_row(self):
        if STEER_STRAIGHT_MIN <= self.aci <= STEER_STRAIGHT_MAX:
            rospy.loginfo("Vehicle aligned on second row.")
            self.durDumen()
            self.move_forward(100)
            self.duraktif = 1
            self.bitkiburda = 0
            self.timer_bulma = 201
        elif 1 < self.aci < STEER_STRAIGHT_MIN:
            self.acison = 1
            self.sag(100)
            self.move_forward(100)
            self.duraktif = 1
            self.bitkiburda = 1
            self.timer_bulma = 201
        elif self.aci > STEER_STRAIGHT_MAX:
            self.acison = 2
            self.sol(100)
            self.move_forward(100)
            self.duraktif = 1
            self.bitkiburda = 2
            self.timer_bulma = 201
        else:
            if self.duraktif == 1:
                if self.bitkisayi == 0:
                    self.move_forward(70)
                    self.nerdeyim = 4
                else:
                    self.bitki_kaybol()

    def _row_transition(self):
        if self.bisekine == 0:
            self.durDumen()
            self.move_forward(100)
            time.sleep(25)
            rospy.loginfo("Exited row — starting turn.")
            self.bisekine = 16
        if self.bisekine == 16:
            self.dur()
            time.sleep(5)
            rospy.loginfo("Turning for row change.")
            self.sol(100)
            time.sleep(10)
            self.bisekine = 21
        if self.bisekine == 21:
            self.durDumen()
            self.move_forward_2(59, 82)
            time.sleep(13)
            rospy.loginfo("Row transition complete.")

    def bitki_kaybol(self):
        """Recover when plant is temporarily lost."""
        if self.bitkiburda == 1:
            self.sag(70)
            self.move_forward(100)
        elif self.bitkiburda == 2:
            self.sol(70)
            self.move_forward(100)
        else:
            self.move_forward(100)

    # ------------------------------------------------------------------
    # Motor Primitives
    # ------------------------------------------------------------------

    def move_forward(self, speed):
        self.rpwm.ChangeDutyCycle(speed)
        self.lpwm.ChangeDutyCycle(0)
        self.rrsagon.ChangeDutyCycle(speed)
        self.llsagon.ChangeDutyCycle(0)

    def move_forward_2(self, speed_right, speed_left):
        """Differential drive — used during row transition turns."""
        self.rpwm.ChangeDutyCycle(speed_right)
        self.lpwm.ChangeDutyCycle(0)
        self.rrsagon.ChangeDutyCycle(speed_left)
        self.llsagon.ChangeDutyCycle(0)

    def move_backward(self, speed):
        self.rpwm.ChangeDutyCycle(0)
        self.lpwm.ChangeDutyCycle(speed)
        self.rrsagon.ChangeDutyCycle(0)
        self.llsagon.ChangeDutyCycle(speed)

    def sol(self, speed):
        """Steer left (rudder motor)."""
        self.rpwmdumen.ChangeDutyCycle(0)
        self.lpwmdumen.ChangeDutyCycle(speed)

    def sag(self, speed):
        """Steer right (rudder motor)."""
        self.rpwmdumen.ChangeDutyCycle(speed)
        self.lpwmdumen.ChangeDutyCycle(0)

    def dur(self):
        """Stop all drive motors."""
        self.rpwm.ChangeDutyCycle(0)
        self.lpwm.ChangeDutyCycle(0)
        self.rrsagon.ChangeDutyCycle(0)
        self.llsagon.ChangeDutyCycle(0)
        time.sleep(0.1)

    def durDumen(self):
        """Stop steering motor."""
        self.rpwmdumen.ChangeDutyCycle(0)
        self.lpwmdumen.ChangeDutyCycle(0)

    def stop(self):
        """Stop and release all PWM channels."""
        for pwm in [self.rpwm, self.lpwm, self.rrsagon, self.llsagon,
                    self.rpwmdumen, self.lpwmdumen]:
            pwm.stop()


if __name__ == "__main__":
    rospy.init_node("motor_controller", anonymous=True)
    motor_controller = MotorController()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        motor_controller.stop()
        GPIO.cleanup()
