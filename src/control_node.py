#!/usr/bin/env python3

import rospy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt8

from scheduling.configs import *

import numpy as np
import math
import time

from enum import Enum

class State(Enum):
    IDLE = 0
    ASSIGNED = 1

class ControlNode:
    def __init__(self):
        rospy.init_node("control_node")

        # Publishers
        self.pub_vel = rospy.Publisher(rospy.get_namespace()+VEL_TOPIC, Twist, queue_size=CONTROL_RATE)
        self.pub_state = rospy.Publisher(rospy.get_namespace()+STATE_TOPIC, UInt8, queue_size=CONTROL_RATE)

        # Subscribers
        rospy.Subscriber(rospy.get_namespace()+ODOMETRY_TOPIC, Odometry, self.odomCallback)
        rospy.Subscriber(rospy.get_namespace()+GOAL_TOPIC, PoseStamped, self.goalCallback)

        self.state = State.IDLE
        self.odom = Odometry()
        self.goal = PoseStamped()

        self.rate = rospy.Rate(CONTROL_RATE)

    def odomCallback(self, data:Odometry):
        self.odom = data

    def goalCallback(self, data:PoseStamped):
        self.goal = data

    def publishState(self):
        msg_state = UInt8()
        msg_state.data = self.state.value
        self.pub_state.publish(msg_state)

    def assignmentWaiting(self):
        if self.goal != PoseStamped():
            self.state = State.ASSIGNED

    def assignmentPerforming(self):
        pass

    def execute(self):
        while not rospy.is_shutdown():
            if self.state == State.IDLE:
                self.assignmentWaiting()
            elif self.state == State.ASSIGNED:
                self.assignmentPerforming()

            # Publish topics
            self.publishState()

            # Sleep
            self.rate.sleep()

if __name__ == "__main__":
    node = ControlNode()
    try:
        node.execute()
    except rospy.ROSInterruptException:
        pass