# Turtlebot 3 Scheduling platform

## Prerequisite
```
$ sudo apt install ros-$ROS_DISTRO-turtlebot3-gazebo
```

## Installation
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ git clone git@github.com:duynamrcv/turtlebot3_scheduling.git
$ cd ..
$ catkin build
```

## Demo
### For single robot
Launch Turtlebot3 robot
```
$ roslaunch turtlebot3_scheduling single.launch
```
Assign next target
```
$ rosservice call /next_goal "goal:
  position:
    x: 1.0
    y: 0.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0" 

```

### For multiple robots
To be defined

