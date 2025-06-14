#!/usr/bin/python3
import os
import time
from rich import print
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelEncoderStamped

from std_msgs.msg import Float32MultiArray

# ticks_per_decimeter = 65
ticks_per_revolution = 135

class EncoderNode(DTROS):
    def __init__(self, node_name):
        super(EncoderNode, self).__init__(
            node_name=node_name, node_type=NodeType.GENERIC)
        
        self.publisher = rospy.Publisher("encoders", Float32MultiArray, queue_size=1)

        self.ticks = [0, 0]
        self.previous = [None, None]
        self.p_time = [time.time(), time.time()]

        robot_name: str = os.environ['VEHICLE_NAME']

        left_topic = f"/{robot_name}/left_wheel_encoder_node/tick"
        right_topic = f"/{robot_name}/right_wheel_encoder_node/tick"

        rospy.Subscriber(left_topic, WheelEncoderStamped, self.callback_left)
        rospy.Subscriber(right_topic, WheelEncoderStamped, self.callback_right)

    def callback_left(self, msg):
        if self.previous[0] is None:
            self.previous[0] = msg.data

        new_time = time.time()
        delta_time = new_time - self.p_time[0]
        self.p_time[0] = new_time

        self.ticks[0] = float(msg.data - self.previous[0]) / ticks_per_revolution / delta_time
        self.previous[0] = msg.data

    def callback_right(self, msg):
        if self.previous[1] is None:
            self.previous[1] = msg.data
        
        new_time = time.time()
        delta_time = new_time - self.p_time[1]
        self.p_time[1] = new_time

        self.ticks[1] = float(msg.data - self.previous[1]) / ticks_per_revolution / delta_time
        self.publisher.publish(Float32MultiArray(data=self.ticks))
        self.previous[1] = msg.data


if __name__ == '__main__':
    node = EncoderNode(node_name='encoder_node')
    # node.run()
    rospy.spin()
