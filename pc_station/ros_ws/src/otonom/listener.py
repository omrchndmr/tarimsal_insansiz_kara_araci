import sys
import time
from pymata4 import pymata4
import rospy
from std_msgs.msg import Float32

# Sensör pin numaraları
TRIGGER_PINS = [3, 5, 8, 10]  # Trigger pin numaraları
ECHO_PINS = [4, 6, 9, 11]     # Echo pin numaraları

# ROS yayıncıları için topic isimleri
TOPIC_NAMES = [f'sensor_{i}_distance' for i in range(4)]

def the_callback(sensor_index):
    def callback(data):
        distance = data[2]  # Mesafe verisi
        rospy.loginfo(f'Sensor {sensor_index} Distance in cm: {distance}')
        publishers[sensor_index].publish(distance)
    return callback

def sonar(my_board, trigger_pins, echo_pins, callbacks):
    for i in range(len(trigger_pins)):
        my_board.set_pin_mode_sonar(trigger_pins[i], echo_pins[i], callbacks[i])

    while not rospy.is_shutdown():
        try:
            time.sleep(0.5)
        except KeyboardInterrupt:
            my_board.shutdown()
            sys.exit(0)

if __name__ == '__main__':
    rospy.init_node('sonar_node', anonymous=True)

    # ROS yayıncıları
    publishers = [rospy.Publisher(topic, Float32, queue_size=10) for topic in TOPIC_NAMES]

    board = pymata4.Pymata4()

    # Sensörler için callback fonksiyonları
    callbacks = [the_callback(i) for i in range(4)]

    try:
        sonar(board, TRIGGER_PINS, ECHO_PINS, callbacks)
        board.shutdown()
    except (KeyboardInterrupt, RuntimeError):
        board.shutdown()
        sys.exit(0)

