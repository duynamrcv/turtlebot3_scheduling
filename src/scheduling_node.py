#!/usr/bin/env python3

import rospy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt8
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from scheduling.configs import *
from turtlebot3_scheduling.srv import NextPose

import numpy as np
import math
import time

class SchedulingNode:
    def __init__(self):
        rospy.init_node("scheduling_node")
        rospy.wait_for_service(rospy.get_namespace()+NEXTGOAL_TOPIC)

        # Declare publisher/subscriber topics in here

        self.rate = rospy.Rate(CONTROL_RATE)

    def sendNextGoal(self, goal:Pose):
        send_next_goal = rospy.ServiceProxy(rospy.get_namespace()+NEXTGOAL_TOPIC, NextPose)
        success = send_next_goal(goal)
        if success:
            rospy.loginfo("Assign goal to robot successful")
        else:
            rospy.loginfo("Assign failed")

    def execute(self):
        while not rospy.is_shutdown():
            # Do something in here

            next_goal = np.array([2,1,np.pi/3]) # [x, y, yaw]
            
            # Send goal to robot
            goal = Pose()
            goal.position.x = next_goal[0]
            goal.position.y = next_goal[1]
            quat = quaternion_from_euler(0, 0, next_goal[2])
            goal.orientation.x = quat[0]
            goal.orientation.y = quat[1]
            goal.orientation.z = quat[2]
            goal.orientation.w = quat[3]
            self.sendNextGoal(goal)

            # Sleep
            self.rate.sleep()

if __name__ == "__main__":
    node = SchedulingNode()
    try:
        node.execute()
    except rospy.ROSInterruptException:
        pass