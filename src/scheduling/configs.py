import numpy as np
MODEL = "burger"
CONTROL_RATE = 20
EPSILON = 0.05

# Control parameters
KP_RHO = .9
KP_ALPHA = 1.5
KP_BETA = -.3

MAX_V = 1.0
MAX_W = np.pi/3

# Publish topics
STATE_TOPIC = "state"           # std_msgs.msg/UInt8
VEL_TOPIC = "cmd_vel"           # geometry_msgs/Twist

# Subscribe topics
ODOMETRY_TOPIC = "odom"         # nav_msgs/Odometry
LIDAR_TOPIC = "scan"            # sensor_msgs/LaserScan

# Service-Client topics
NEXTGOAL_TOPIC = "next_goal"    # turtlebot3_scheduling/NextPose