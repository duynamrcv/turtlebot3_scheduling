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

## Explanation
###  State Diagrams
```mermaid
stateDiagram
    [*] --> IDLE
    IDLE --> [*]
    IDLE --> ASSIGNED
    ASSIGNED --> IDLE
```
* ```IDLE``` state to be ready to receive the task of moving to the next target.
* ```ASSIGNED``` state to make a move to the next target without being affected by further assignment.

### Moving to pose
Given the robot position $p=\left[x, y, \theta\right]^T$ and the goal position $p_g=\left[x_g, y_g, \theta_g\right]^T$

Firstly, the different position is given as follows:
```math
x_d=x_g-x
```
```math
y_d=y_g-y
```

Convert to the polar coordination
```math
\rho = \sqrt{x_d^2+y_d^2}
```
```math
\alpha = \tan^{-1}\dfrac{y_d}{x_d} - \theta
```
```math
\beta = \theta_d - \theta - \alpha
```

The control signal therefore can be computed as follows:
```math
v = k_\rho\rho
```
```math
w = k_\alpha\alpha + k_\beta\beta
```

Ensure the steering capacity: $\alpha > \pi /2$ or $\alpha < -\pi/2$, revert the velocity direction $v = -v$