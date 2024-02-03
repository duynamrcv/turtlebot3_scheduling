#!/usr/bin/env python3

import rospy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt8
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from scheduling.configs import *
from turtlebot3_scheduling.srv import NextPose, NextPoseResponse

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

        # Service Clients
        rospy.Service(rospy.get_namespace()+NEXTGOAL_TOPIC, NextPose, self.goalCallback)

        self.state = State.IDLE
        self.odom = Odometry()
        self.goal_queue = []
        self.goal_current = None

        self.rate = rospy.Rate(CONTROL_RATE)

    def odomCallback(self, data:Odometry):
        self.odom = data

    def goalCallback(self, req):
        rospy.loginfo("Received new assignment")
        euler = euler_from_quaternion([req.goal.orientation.x,
                                       req.goal.orientation.y,
                                       req.goal.orientation.z,
                                       req.goal.orientation.w])
        next_goal = np.array([req.goal.position.x, req.goal.position.y, euler[2]]) # [x, y, yaw]
        self.goal_queue.append(next_goal)
        return NextPoseResponse(True)

    def publishState(self):
        msg_state = UInt8()
        msg_state.data = self.state.value
        self.pub_state.publish(msg_state)

    def assignmentWaiting(self):
        if self.goal_current is None and self.goal_queue != []:
            self.goal_current = self.goal_queue.pop(0)
            rospy.loginfo("Moving to next goal: x = {}; y = {}; yaw = {}".format(self.goal_current[0],
                                                                                 self.goal_current[1],
                                                                                 self.goal_current[2]))
            self.state = State.ASSIGNED

    def assignmentPerforming(self):
        # Check reached goal
        error = np.hypot(self.odom.pose.pose.position.x - self.goal_current[0],
                         self.odom.pose.pose.position.y - self.goal_current[1])
        if error <= EPSILON:
            self.goal_current = None
            self.state = State.IDLE
            self.stopRobot()
        else:
            self.computeControlSignal()
    
    def stopRobot(self):
        # Publish control signal
        vel_msg = Twist()
        self.pub_vel.publish(vel_msg)

    def computeControlSignal(self):
        # Get current robot position
        euler = euler_from_quaternion([self.odom.pose.pose.orientation.x,
                                       self.odom.pose.pose.orientation.y,
                                       self.odom.pose.pose.orientation.z,
                                       self.odom.pose.pose.orientation.w])
        pose_current = np.array([self.odom.pose.pose.position.x,
                                self.odom.pose.pose.position.y,
                                euler[2]])
        
        # Control signal
        x_diff = self.goal_current[0] - pose_current[0]
        y_diff = self.goal_current[1] - pose_current[1]

        rho = np.hypot(x_diff, y_diff)
        alpha = (np.arctan2(y_diff, x_diff) - pose_current[2] + np.pi)%(2*np.pi) - np.pi
        beta = (self.goal_current[2] - pose_current[2] - alpha + np.pi) % (2 * np.pi) - np.pi

        v = KP_RHO*rho
        w = KP_ALPHA*alpha + KP_BETA*beta

        if alpha > np.pi /2 or alpha < -np.pi/2:
            v = -v
        
        # Normalization
        v = min(v, MAX_V)
        w = min(MAX_W, max(w, -MAX_W))

        # Publish control signal
        vel_msg = Twist()
        vel_msg.linear.x = v
        vel_msg.angular.z = w
        self.pub_vel.publish(vel_msg)

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