MODEL = "burger"
CONTROL_RATE = 20
EPSILON = 0.05

# Publish topics
STATE_TOPIC = "state"           # std_msgs.msg/UInt8
VEL_TOPIC = "cmd_vel"           # geometry_msgs/Twist

# Subscribe topics
ODOMETRY_TOPIC = "odom"         # nav_msgs/Odometry
LIDAR_TOPIC = "scan"            # sensor_msgs/LaserScan
GOAL_TOPIC = "goal"             # geometry_msgs/PoseStamped