# Catchrobo 2021 ROS Package

## Environment
- ubuntu 18
- ros melodic

## Requirement


## How to use
### Launching simulated robot
1. Launch moveit
```
roslaunch catchrobo_description catchrobo_display.launch gui:=True field:=red

```

### mbed simulator demo
```
roslaunch catchrobo_test test.launch

rostopic pub /my_joint_control catchrobo_msgs/LearRobotControl "mode: ''
position: [0.5, 0.1, 0.1]
velocity: [0]
effort: [0]
position_limit: [0]
velocity_limit: [0.5, 0.1, 0.1]
acceleration_limit: [0.1, 0.1, 0.1]
jerk_limit: [0.1, 0.1, 0.1]
kp: [0]
kd: [0]" 


```
