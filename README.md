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
rosrun catchrobo_test mbed_sim_test.py # commandをpublishするだけのテストドライバ。自由に書き換え可能
```

シミュレーターの位置更新は下の式で実行される。kp, kdの値を変えるとそれっぽい変化をする.
```
p += cmd.kp * (cmd.p - p) + cmd.kd * cmd.v * dt
```
