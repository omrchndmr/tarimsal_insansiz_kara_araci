#!/usr/bin/env python3
"""
Plant detection node using YOLOv8.

Subscribes to a USB camera image stream, detects crop plants using a YOLOv8
model, computes the midpoint between the two closest (highest-y) detected
plants, and publishes the steering angle and plant positions for downstream
motor control.

ROS Topics Published:
    /object_positions  (geometry_msgs/Point): Midpoint x-coordinate of two plants
    /object_positions_y (geometry_msgs/Point): Midpoint y-coordinate of two plants
    /left_angle        (std_msgs/Float32): Steering angle
    /right_angle       (std_msgs/Float32): Right angle (reserved)
    /single_object_angle (std_msgs/Float32): Angle for single-object mode (reserved)
    /detected_plant    (std_msgs/Float32): 1.0 if plant detected, 0.0 otherwise

ROS Topics Subscribed:
    /usb_cam/image_raw (sensor_msgs/Image): Raw camera image
    /ortalama          (std_msgs/Float32): Averaging mode selector
"""

import math
import os

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from ultralytics import YOLO

# ---------------------------------------------------------------------------
# Configuration — override via ROS parameters or environment variables
# ---------------------------------------------------------------------------
DEFAULT_MODEL_PATH = os.environ.get(
    "YOLO_MODEL_PATH",
    os.path.join(os.path.dirname(__file__), "..", "models", "best.pt"),
)
CONFIDENCE_THRESHOLD = 0.35
LEFT_WHEEL_X = 280   # Pixel x-coordinate of the left wheel reference point
RIGHT_WHEEL_X = 400  # Pixel x-coordinate of the right wheel reference point
STEER_ORIGIN_X = 340  # x-coordinate of the steering reference point
MIN_HORIZONTAL_DISTANCE = 100  # Minimum pixel gap between left/right plants
MAX_VERTICAL_DISTANCE = 105    # Maximum pixel height difference between plants
PUBLISH_INTERVAL_S = 0.2       # Timer interval for publishing positions
DETECT_INTERVAL_S = 0.4        # Timer interval for plant-detected flag


class ObjectDetector:
    """Detects crop rows with YOLOv8 and publishes steering guidance via ROS."""

    def __init__(self):
        model_path = rospy.get_param("~model_path", DEFAULT_MODEL_PATH)
        rospy.loginfo(f"Loading YOLO model from: {model_path}")
        self.model = YOLO(model_path)

        self.bridge = CvBridge()
        self.ortalama = 2

        # State variables
        self.last_left_angle = None
        self.last_right_angle = None
        self.midpoint_x = None
        self.midpoint_y = None
        self.midpoint_x1 = None
        self.single_object_angle = None
        self.detected_two_plants = False
        self.plant_z_distances = []
        self.bitkisayi = 0

        # Subscribers
        self.image_sub = rospy.Subscriber(
            "/usb_cam/image_raw", Image, self.image_callback
        )
        self.ortalama_sub = rospy.Subscriber(
            "/ortalama", Float32, self.ortalama_callback
        )

        # Publishers
        self.position_pub = rospy.Publisher(
            "/object_positions", Point, queue_size=10
        )
        self.positiony_pub = rospy.Publisher(
            "/object_positions_y", Point, queue_size=10
        )
        self.left_angle_pub = rospy.Publisher(
            "/left_angle", Float32, queue_size=10
        )
        self.right_angle_pub = rospy.Publisher(
            "/right_angle", Float32, queue_size=10
        )
        self.single_object_angle_pub = rospy.Publisher(
            "/single_object_angle", Float32, queue_size=10
        )
        self.detected_plant_pub = rospy.Publisher(
            "/detected_plant", Float32, queue_size=10
        )

        # Timers
        self.publish_timer = rospy.Timer(
            rospy.Duration(PUBLISH_INTERVAL_S), self.timer_callback
        )
        self.detect_timer = rospy.Timer(
            rospy.Duration(DETECT_INTERVAL_S), self.bitkitespit
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def ortalama_callback(self, msg):
        self.ortalama = msg.data
        self.last_ortalama_time = rospy.Time.now()

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as exc:
            rospy.logerr(f"CvBridge error: {exc}")
            return

        results = self.model(cv_image)

        object_centers = []
        detection_scores = []
        class_ids = []
        self.bitkisayi = []

        # Track topmost/bottommost plants on each side for potential future use
        topmost_left_plant = {"x1": None, "y1": float("inf"), "x2": None, "y2": None}
        bottommost_left_plant = {"x1": None, "y1": None, "x2": None, "y2": float("-inf")}
        topmost_right_plant = {"x1": None, "y1": float("inf"), "x2": None, "y2": None}
        bottommost_right_plant = {"x1": None, "y1": None, "x2": None, "y2": float("-inf")}

        image_center_x = cv_image.shape[1] // 2

        for result in results:
            boxes = result.boxes.xyxy.cpu().numpy()
            confidences = result.boxes.conf.cpu().numpy()
            class_ids_array = result.boxes.cls.cpu().numpy()

            for i, box in enumerate(boxes):
                x1, y1, x2, y2 = box.astype(int)

                if confidences[i] <= CONFIDENCE_THRESHOLD:
                    continue

                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)
                object_centers.append((cx, cy))
                detection_scores.append(confidences[i])
                class_ids.append(int(class_ids_array[i]))
                self.bitkisayi = object_centers

                if cx < image_center_x:  # Left side
                    if y1 < topmost_left_plant["y1"]:
                        topmost_left_plant = {"x1": x1, "y1": y1, "x2": x2, "y2": y2}
                    if y2 > bottommost_left_plant["y2"]:
                        bottommost_left_plant = {"x1": x1, "y1": y1, "x2": x2, "y2": y2}
                    cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 255), 2)
                else:  # Right side
                    if y1 < topmost_right_plant["y1"]:
                        topmost_right_plant = {"x1": x1, "y1": y1, "x2": x2, "y2": y2}
                    if y2 > bottommost_right_plant["y2"]:
                        bottommost_right_plant = {"x1": x1, "y1": y1, "x2": x2, "y2": y2}

                cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                label = (
                    f"Class: {int(class_ids_array[i])}, "
                    f"Conf: {confidences[i]:.2f}"
                )
                cv2.putText(
                    cv_image, label, (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2,
                )

        if len(object_centers) >= 2 and self.ortalama == 2:
            top_leftmost, top_rightmost = self._find_highest_y_objects(object_centers)

            if top_leftmost and top_rightmost:
                leftmost = min(object_centers, key=lambda p: p[0])
                rightmost = max(object_centers, key=lambda p: p[0])

                horiz_dist = abs(leftmost[0] - rightmost[0])
                vert_dist = abs(leftmost[1] - rightmost[1])

                if horiz_dist >= MIN_HORIZONTAL_DISTANCE and vert_dist < MAX_VERTICAL_DISTANCE:
                    self.midpoint_x1 = (leftmost[0] + rightmost[0]) / 2
                    self.midpoint_y = (top_leftmost[1] + top_rightmost[1]) / 2

                    fixed_line = ((LEFT_WHEEL_X, 479), (RIGHT_WHEEL_X, 479))
                    steer_line = (
                        (STEER_ORIGIN_X, 479),
                        (self.midpoint_x1, self.midpoint_y),
                    )
                    self.last_left_angle = self.calculate_angle(
                        fixed_line, steer_line, self.midpoint_x1
                    )

                    cv2.putText(
                        cv_image,
                        f"Angle: {self.last_left_angle:.2f} deg",
                        (50, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2,
                    )
                    cv2.line(
                        cv_image,
                        (STEER_ORIGIN_X, 479),
                        (int(self.midpoint_x1), int(self.midpoint_y)),
                        (0, 0, 255), 2,
                    )

            self.detected_two_plants = True

        # Draw reference baseline
        cv2.line(cv_image, (LEFT_WHEEL_X, 479), (RIGHT_WHEEL_X, 479), (0, 0, 255), 2)
        cv2.imshow("Plant Detection", cv_image)
        cv2.waitKey(3)

    def timer_callback(self, event):
        if self.ortalama == 2:
            self.position_pub.publish(Point(self.midpoint_x1, 0, 0))
            self.positiony_pub.publish(Point(0, self.midpoint_y, 0))

        self.left_angle_pub.publish(Float32(self.last_left_angle))
        self.right_angle_pub.publish(Float32(self.last_right_angle))

        # Reset angles so stale values are not re-published on the next cycle
        self.last_left_angle = None
        self.last_right_angle = None

    def bitkitespit(self, event):
        if isinstance(self.bitkisayi, list) and len(self.bitkisayi) > 0:
            self.detected_plant_pub.publish(Float32(1.0))
        else:
            self.detected_plant_pub.publish(Float32(0.0))

    # ------------------------------------------------------------------
    # Geometry helpers
    # ------------------------------------------------------------------

    def _find_highest_y_objects(self, object_centers):
        """Return the two objects with the largest y-coordinate (closest to camera)."""
        sorted_objects = sorted(object_centers, key=lambda obj: obj[1], reverse=True)
        if len(sorted_objects) >= 2:
            return sorted_objects[0], sorted_objects[1]
        if len(sorted_objects) == 1:
            return sorted_objects[0], None
        return None, None

    @staticmethod
    def _slope(line):
        (x1, y1), (x2, y2) = line
        return float("inf") if x2 == x1 else (y2 - y1) / (x2 - x1)

    def calculate_angle(self, line1, line2, x_position=None):
        """Compute the acute angle (degrees) between two lines."""
        s1 = self._slope(line1)
        s2 = self._slope(line2)

        if s1 == float("inf"):
            angle = 90.0 if s2 != float("inf") else 0.0
        elif s2 == float("inf"):
            angle = 90.0
        else:
            tan_angle = abs((s2 - s1) / (1 + s1 * s2))
            angle = math.degrees(math.atan(tan_angle))

        if x_position is not None and x_position < STEER_ORIGIN_X:
            angle = 180.0 - angle

        return angle


if __name__ == "__main__":
    rospy.init_node("object_detector", anonymous=True)
    try:
        ObjectDetector()
        rospy.spin()
    except KeyboardInterrupt:
        cv2.destroyAllWindows()
