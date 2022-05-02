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


rostopic pub /my_joint_control catchrobo_msgs/MyRosCmdArray "command_array:
- {mode: 0, position: 0.0, velocity: 0.0, effort: 0.0, position_limit: 0.0, velocity_limit: 0.5,
  acceleration_limit: 0.3, jerk_limit: 0.1, kp: 0.0, kd: 0.0}
- {mode: 0, position: 0.0, velocity: 0.0, effort: 0.0, position_limit: 0.0, velocity_limit: 0.5,
  acceleration_limit: 0.3, jerk_limit: 0.1, kp: 0.0, kd: 0.0}
- {mode: 0, position: 0.0, velocity: 0.0, effort: 0.0, position_limit: 0.0, velocity_limit: 0.5,
  acceleration_limit: 0.3, jerk_limit: 0.1, kp: 0.0, kd: 0.0}
  " 

rostopic pub /my_joint_control catchrobo_msgs/MyRosCmdArray "command_array:
- {mode: 0, position: 1.5, velocity: 0.0, effort: 0.0, position_limit: 0.0, velocity_limit: 0.5,
  acceleration_limit: 0.3, jerk_limit: 0.1, kp: 0.0, kd: 0.0}
- {mode: 0, position: 0.0, velocity: 0.0, effort: 0.0, position_limit: 0.0, velocity_limit: 0.5,
  acceleration_limit: 0.3, jerk_limit: 0.1, kp: 0.0, kd: 0.0}
- {mode: 0, position: 0.0, velocity: 0.0, effort: 0.0, position_limit: 0.0, velocity_limit: 0.5,
  acceleration_limit: 0.3, jerk_limit: 0.1, kp: 0.0, kd: 0.0}
  " 


```
