#!/usr/bin/env python3
import time
import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import WheelsCmdStamped
import numpy as np
import cv2
from cv_bridge import CvBridge
from std_msgs.msg import Float64
from collections import deque

from Dlite import *


class CameraReaderNode(DTROS):
    def __init__(self, node_name):
        super(CameraReaderNode, self).__init__(
            node_name=node_name, node_type=NodeType.GENERIC)

        self.actions = {
            "F1": self.follow_lane,
            "F2": self.follow_lane,
            "R1": self.turn_right1,
            "R2": self.turn_right2,
            "R3": self.turn_right2,
            "R4": self.turn_right3,
            "S": self.go_straight,
            "L1": self.turn_left1,
            "L2": self.turn_left2,
            "L3": self.turn_left2,
            "L4": self.turn_left3,
            "I": self.initial_setup,
            "D": self.done,
            "P": self.done,
        }

        # in seconds
        self.action_times = {
            "I":    1, # Initial
            "F1":   1,
            "F2":  -1, # Follow lane
            "S":    2, # Go straight
            "R1": 1.1, # Turn right
            "R2":  -1, # Turn right
            "R3":  -1, # Turn right
            "R4": 0.3,
            "L1": 1.7, # Turn right
            "L2":  -1, # Turn right
            "L3":  -1, # Turn left
            "L4": 1.1, # Turn left
            "P":    1,
            "D": 1000,
        }
        start_v = "v7"
        end_v = "v3"

        self.ind = 0
        self.instructions = self.get_path(start_v, end_v)
        self.instructions = ["I", "F2"] + self.instructions + ["D"]
        
        ni = []
        for s in self.instructions:
            if s == "R":
                ni += ["R1", "R2", "R3", "P", "R4"]
            elif s == "L":
                ni += ["L1", "L2", "L3", "P", "L4"]
            elif s == "F":
                ni += ["F1", "F2", "P"]
            else:
                ni += [s]
        self.instructions = ni
        print(ni)
        self.action_timer = self.action_times[self.instructions[self.ind]]
        self.p_time = time.time()
        self.p_left_motor = 0
        self.p_right_motor = 0

        self.red = None
        self.should_check_red = False

        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        wheels_topic = f"/{self._vehicle_name}/wheels_driver_node/wheels_cmd"
        self._window = "camera-reader"
        self.bridge = CvBridge()
        cv2.namedWindow(self._window, cv2.WINDOW_AUTOSIZE)

        self.image = None
        self.sub = rospy.Subscriber(
            self._camera_topic, CompressedImage, self.callback)
        self._publisher = rospy.Publisher(
            wheels_topic, WheelsCmdStamped, queue_size=1)

        self.left_motor = rospy.Publisher("left_motor", Float64, queue_size=1)
        self.right_motor = rospy.Publisher("right_motor", Float64, queue_size=1)

        self.shutting_down = False
        rospy.on_shutdown(self.shutdown_hook)

    def get_path(self, start_v, end_v):
        graph = create_graph("map.txt")
        d_star = DStarLite(start_v, end_v, graph)
        d_star.compute_shortest_path()
        return d_star.get_instruction_sequence()

    def shutdown_hook(self):
        self.shutting_down = True
        self.left_motor.publish(0)
        self.right_motor.publish(0)
        cv2.destroyAllWindows()

    def callback(self, msg):
        if self.ind >= len(self.instructions):
            return
        self.image = self.bridge.compressed_imgmsg_to_cv2(msg)

    def run(self):
        rate = rospy.Rate(15)

        frame_count = 0
        while not self.shutting_down:
            rate.sleep()

            if self.image is None:
                continue

            self.vis_image = self.image.copy()

            self.delta_time = time.time() - self.p_time
           self.p_time = time.time()
            self.action_timer -= self.delta_time

            action_end = False
            if self.instructions[self.ind] == "F2":
                action_end = self.detect_red()
            elif self.instructions[self.ind] in ["L2", "R2"]:
                action_end = not self.sees_a_lane()
            elif self.instructions[self.ind] in ["L3", "R3"]:
                action_end = self.sees_a_lane()
            else:
                action_end = self.action_timer < 0
                
            if action_end:
                self.ind += 1
                if self.ind >= len(self.instructions):
                    return
                self.action_timer = self.action_times[self.instructions[self.ind]]

            base_speed = 0.5 if self.instructions[self.ind] in ["F1", "F2"] else 0.4
            left_motor, right_motor = self.actions[self.instructions[self.ind]]()
            left_motor *= base_speed
            right_motor *= base_speed

            self.p_left_motor = lerp(self.p_left_motor, left_motor, 1)
            self.p_right_motor = lerp(self.p_right_motor, right_motor, 1)

            self.left_motor.publish(self.p_left_motor)
            self.right_motor.publish(self.p_right_motor)

            self.visualization()
            frame_count += 1

    def done(self):
        return 0, 0

    def detect_red(self):
        h, w = self.image.shape[:2]
        oi, oj = int(h*0.9), int(w*0.4)
        h, w = h - oi, int(w * 0.6) - int(w * 0.4)
        roi = self.image[oi:oi+h, oj:oj+w]

        d = np.array([30, 30, 30])
        mask = cv2.inRange(roi, self.red - d, self.red + d)

        return np.mean(mask, axis=(0, 1)) > 0.5

    def initial_setup(self):
        self.red = self.find_ground_color()
        return 0.0, 0.0

    def turn_right1(self):
        return 1.0, 1.0
    
    def turn_right2(self):
        return 0.5, -0.5
    
    def turn_right3(self):
        return 1.0, 1.0

    def turn_left1(self):
        return 1.0, 1.0
    
    def turn_left2(self):
        return -0.5, 0.5
    
    def turn_left3(self):
        return 1.0, 1.0

    def go_straight(self):
        return 1.0, 1.0

    def follow_lane(self):
        img_h, img_w = self.image.shape[:2]

        roi_w, roi_h = int(0.64 * img_w), int(0.47 * img_h)

        roi_y, roi_x = img_h - roi_h, (img_w - roi_w) // 2

        roi = self.image[roi_y:roi_y+roi_h, roi_x:roi_x+roi_w]
        roi_h //= 3
        roi_w //= 3
        roi = cv2.resize(roi, (roi_w, roi_h))
        
        edges = cv2.Canny(roi, 100, 300).astype(dtype=np.uint8)

        center_width = 10

        ignore_tolerance = 5

        ab = (roi_w - center_width) // 2
        bc = (roi_w + center_width) // 2
        center_part = np.any(edges[::-1, ab:bc+1], axis=1)
        ind = min(int(roi_h * 0.2), roi_h - np.argmax(center_part, axis=0) + ignore_tolerance) if np.any(center_part) else 0
        edges[:ind, :] = np.zeros((ind, roi_w))

        self.should_check_red = ind > roi_h * 0.5

        self.vis_image[roi_y:roi_y+roi_h, roi_x:roi_x+roi_w] = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
        cv2.rectangle(self.vis_image, (roi_x + ab, roi_y), (roi_x + bc, roi_y + roi_h), (0, 255, 0), 1)

        x, y = centre_of_mass_lr(edges, 0.5, 1.0)

        ind = min(roi_h, roi_h - np.argmax(np.any(edges, axis=1), axis=0)) if np.any(edges, axis=(0, 1)) else roi_h

        def clip(a, l, h):
            return min(max(a, l), h)

        clip_factor = ind / roi_h * 0.2 + 0.8
        steer = clip((1 - 2 * x / roi_w) * 1.1, -clip_factor, clip_factor)

        base = (ind / roi_h) * 1.2 - 0.2
        return base + min(steer, 0), base - max(steer, 0)

    def sees_a_lane(self):
        img_h, img_w = self.image.shape[:2]

        roi_w, roi_h = 21, int(0.50 * img_h)

        roi_y, roi_x = img_h - roi_h, (img_w - roi_w) // 2

        roi = self.image[roi_y:roi_y+roi_h, roi_x:roi_x+roi_w]
        roi_h //= 3
        roi_w //= 3
        roi = cv2.resize(roi, (roi_w, roi_h))
        
        edges = cv2.Canny(roi, 100, 300).astype(dtype=np.uint8)

        return not np.any(edges, axis=(0, 1))

    def find_ground_color(self):
        h, w = self.image.shape[:2]
        roi = self.image[h-11:h-1, w//2-10:w//2+10]
        return np.mean(roi, axis=(0,1))

    def visualization(self):
        yoff = 30

        def write(t):
            nonlocal yoff
            
            cv2.putText(self.vis_image, t, (10, yoff), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            yoff += 30

        write(f"Current Instruction: {self.instructions[self.ind]} (next: {self.instructions[self.ind + 1] if self.ind < len(self.instructions) - 1 else 'end'})")
        write(f"instruction index: {self.ind}")
        # write(f"action timer: {self.action_timer}")
        # write(f"{self.image[0][0][0]}")
        # write(f"{self.image[0][0][1]}")
        # write(f"{self.image[0][0][2]}")
        write(f"{self.p_left_motor}")
        write(f"{self.p_right_motor}")
        # is_red = "yes" if self.detect_red() else "no"
        # write(f"shold check red? {self.should_check_red}")
        # write(f"is red? {is_red}")
        # write(f"sees a lane? {self.sees_a_lane()}")
        
        h, w = self.image.shape[:2]
        f = self.p_right_motor + self.p_left_motor
        x = w * (self.p_right_motor / f if f != 0 else 0.5)
        # cv2.circle(self.vis_image, (int(x), h//3), 5, 126, 5)
        
        cv2.imshow(self._window, self.vis_image)
        cv2.waitKey(1)


def magic_mask(w, h, t, b, l, r):
    ttb, _ = np.mgrid[0:h:1, 0:w:1] * (b - t) + t
    res = np.zeros((h, w))
    res[:, :w//2] = ttb[:, :w//2] * l
    res[:, w//2:] = ttb[:, w//2:] * r
    return res


def mask_generation(w, h, tri_h):
    res = np.zeros((h, w), dtype=np.uint8)
    for i in range(h):
        for j in range(w // 2):
            res[i, j] = 0 if j == 0 or (h - i - 1) / j > tri_h / (w // 2) else 1
            res[i, w - j - 1] = res[i, j]
    return res


def centre_of_mass_lr(mask, l, r):
    h = mask.shape[1] // 2
    Y_l, X_l = np.mgrid[0:mask.shape[0]:1, 0:h:1]
    Y_r, X_r = np.mgrid[0:mask.shape[0]:1, h:mask.shape[1]:1]

    if np.count_nonzero(mask):
        mask_l = mask[:, :h]
        mask_r = mask[:, h:]

        centroid_x_l = int(float(np.sum(cv2.bitwise_and(X_l, X_l, mask=mask_l)))) * l
        centroid_y_l = int(float(np.sum(cv2.bitwise_and(Y_l, Y_l, mask=mask_l)))) * l
        centroid_x_r = int(float(np.sum(cv2.bitwise_and(X_r, X_r, mask=mask_r)))) * r
        centroid_y_r = int(float(np.sum(cv2.bitwise_and(Y_r, Y_r, mask=mask_r)))) * r
        centroid_x = (centroid_x_l + centroid_x_r)/(np.count_nonzero(mask_r) * r + np.count_nonzero(mask_l) * l)
        centroid_y = (centroid_y_l + centroid_y_r)/(np.count_nonzero(mask_r) * r + np.count_nonzero(mask_l) * l)
    else:
        centroid_x = mask.shape[1] / 2
        centroid_y = mask.shape[0] / 2

    return(centroid_x, centroid_y)

def distsq(a, b):
    return \
        (a[0] - b[0]) * (a[0] - b[0]) + \
        (a[1] - b[1]) * (a[1] - b[1]) + \
        (a[2] - b[2]) * (a[2] - b[2])

def lerp(a, b, t):
    return a + (b - a) * t

if __name__ == '__main__':
    node = CameraReaderNode(node_name='camera_reader_node')
    node.run()
    rospy.spin() 
