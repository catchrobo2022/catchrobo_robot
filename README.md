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
```

position control : mode 0 and kp 1 kd 0
```
rostopic pub /my_joint_control catchrobo_msgs/MyRosCmdArray "command_array:
- {mode: 0, position: 0.0, velocity: 0.0, effort: 0.0, position_limit: 0.0, velocity_limit: 0.5,
  acceleration_limit: 0.3, jerk_limit: 0.1, kp: 1.0, kd: 0.0}
- {mode: 0, position: 0.0, velocity: 0.0, effort: 0.0, position_limit: 0.0, velocity_limit: 0.5,
  acceleration_limit: 0.3, jerk_limit: 0.1, kp: 0.0, kd: 0.0}
- {mode: 0, position: 0.0, velocity: 0.0, effort: 0.0, position_limit: 0.0, velocity_limit: 0.5,
  acceleration_limit: 0.3, jerk_limit: 0.1, kp: 0.0, kd: 0.0}
  " 

rostopic pub /my_joint_control catchrobo_msgs/MyRosCmdArray "command_array:
- {mode: 0, position: 1.5, velocity: 0.0, effort: 0.0, position_limit: 0.0, velocity_limit: 0.5,
  acceleration_limit: 0.3, jerk_limit: 0.1, kp: 1.0, kd: 0.0}
- {mode: 0, position: 0.0, velocity: 0.0, effort: 0.0, position_limit: 0.0, velocity_limit: 0.5,
  acceleration_limit: 0.3, jerk_limit: 0.1, kp: 0.0, kd: 0.0}
- {mode: 0, position: 0.0, velocity: 0.0, effort: 0.0, position_limit: 0.0, velocity_limit: 0.5,
  acceleration_limit: 0.3, jerk_limit: 0.1, kp: 0.0, kd: 0.0}
  " 
```

velocity control : mode 1 and kp 0 kd 1
```
rostopic pub /my_joint_control catchrobo_msgs/MyRosCmdArray "command_array:
- {mode: 1, position: 0, velocity: 0.1, effort: 0.0, position_limit: 0.0, velocity_limit: 0.5,
  acceleration_limit: 0.3, jerk_limit: 0.1, kp: 0.0, kd: 1.0}
- {mode: 0, position: 0.0, velocity: 0.0, effort: 0.0, position_limit: 0.0, velocity_limit: 0.5,
  acceleration_limit: 0.3, jerk_limit: 0.1, kp: 0.0, kd: 0.0}
- {mode: 0, position: 0.0, velocity: 0.0, effort: 0.0, position_limit: 0.0, velocity_limit: 0.5,
  acceleration_limit: 0.3, jerk_limit: 0.1, kp: 0.0, kd: 0.0}
  " 

rostopic pub /my_joint_control catchrobo_msgs/MyRosCmdArray "command_array:
- {mode: 1, position: 0, velocity: -0.1, effort: 0.0, position_limit: 0.0, velocity_limit: 0.5,
  acceleration_limit: 0.3, jerk_limit: 0.1, kp: 0.0, kd: 1.0}
- {mode: 0, position: 0.0, velocity: 0.0, effort: 0.0, position_limit: 0.0, velocity_limit: 0.5,
  acceleration_limit: 0.3, jerk_limit: 0.1, kp: 0.0, kd: 0.0}
- {mode: 0, position: 0.0, velocity: 0.0, effort: 0.0, position_limit: 0.0, velocity_limit: 0.5,
  acceleration_limit: 0.3, jerk_limit: 0.1, kp: 0.0, kd: 0.0}
  " 
```

シミュレーターの位置更新は下の式で実行される。kp, kdの値を変えるとそれっぽい変化をする.
```
p += kp*(target - p) + kd*v*dt
```
