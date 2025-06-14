#!/usr/bin/env python3
import os
import time
from rich import print
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped
from std_msgs.msg import Float64
from std_msgs.msg import Float32MultiArray


class WheelControlNode(DTROS):
    def __init__(self, node_name):
        super(WheelControlNode, self).__init__(
            node_name=node_name, node_type=NodeType.GENERIC)
        
        self.vehicle_name = os.environ['VEHICLE_NAME']
        wheels_topic = f"/{self.vehicle_name}/wheels_driver_node/wheels_cmd"
        self.p_time = time.time()

        self.target = [0, 0]
        self.value = [0, 0]
        self.p_error = [0, 0]
        self.integral = [0, 0]

        self._publisher = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)

        rospy.Subscriber("left_motor", Float64, self.callback_left)
        rospy.Subscriber("right_motor", Float64, self.callback_right)
        rospy.Subscriber("encoders", Float32MultiArray, self.callback_encoders)

        rospy.on_shutdown(self.on_shutdown)

    def callback_left(self, msg):
        self.target[0] = msg.data

    def callback_right(self, msg):
        self.target[1] = msg.data
    
    def callback_encoders(self, msg):
        self.value = msg.data

        new_time = time.time()
        delta_time = new_time - self.p_time
        self.p_time = new_time
        
        error = [self.target[0] - self.value[0], self.target[1] - self.value[1]]
        derivative = [(error[0] - self.p_error[0]), (error[1] - self.p_error[1])]
        self.integral[0] += error[0] * delta_time
        self.integral[1] += error[1] * delta_time

        p = 0.05
        i = 0.01
        d = 0.02

        motor_speed = [
            p * error[0] + i * self.integral[0] + d * derivative[0],
            p * error[1] + i * self.integral[1] + d * derivative[1],
        ]

        message = WheelsCmdStamped(vel_left=motor_speed[0], vel_right=motor_speed[1])

        self.p_error = error

        message = WheelsCmdStamped(vel_left=self.target[0], vel_right=self.target[1])
        self._publisher.publish(message)

    def on_shutdown(self):
        stop = WheelsCmdStamped(vel_left=0, vel_right=0)
        self._publisher.publish(stop)


if __name__ == '__main__':
    node = WheelControlNode(node_name='wheel_control_node')
    rospy.on_shutdown(node.on_shutdown)
    rospy.spin()
