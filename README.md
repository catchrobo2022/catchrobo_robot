# Catchrobo 2021 ROS Package

## Environment
- ubuntu 18
- ros melodic

## Requirement
```
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt-get install -y ros-melodic-joint-state-controller
sudo apt-get install -y ros-melodic-effort-controllers
sudo apt-get install -y ros-melodic-position-controllers
sudo apt-get install -y ros-melodic-ros-control
sudo apt-get install -y ros-melodic-ros-controllers
```

## How to use
### Launching simulated robot
1. Launch moveit
```
roslaunch catchrobo_description catchrobo_display.launch gui:=True field:=red

```
