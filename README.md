# Catchrobo 2021 ROS Package

## Environment
- ubuntu 18
- ros melodic

## Requirement
- moveit (see : https://moveit.ros.org/install/)
- sudo apt install ros-melodic-joint-state-publisher-gui 
- pip install pandas numpy
- sudo apt install ros-melodic-joy
- sudo apt install ros-melodic-joystick-drivers 
- sudo pip3 install --upgrade odrive 
- sudo apt-get install -y ros-kinetic-jsk-visualization

## How to use
### Moving the simulated robot
1. Launch moveit
```
roslaunch catchrobo_bringup sim_bringup.launch
```
2. Launch game manager
```
roslaunch catchrobo_manager catchrobo_manager.launch
```

### Moving the actual robot
1. Allow access to serial port
```
sudo chmod 666 /dev/ttyUSB0
sudo chmod 666 /dev/ttyACM0
```
2. Bringup robot
```
roslaunch catchrobo_bringup bringup.launch
```
3. Launch manager
```
roslaunch catchrobo_manager catchrobo_manager.launch
```

<!-- 
## IKFast  
- download following by http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/ikfast/ikfast_tutorial.html  (export MYROBOT_NAME=" catchrobo_description/robots/catchrobo")  
- display collada file:
```
$ openrave-robot.py catchrobo_description/robots/catchrobo.dae --info links

name              index parents          
-----------------------------------------
world             0                      
base/base_link    1     world            
base/link_tip     2     base/base_link   
arm/link0         3     base/link_tip    
arm/link1         4     arm/link0        
arm/link2         5     arm/link1        
arm/link3         6     arm/link2        
arm/link4         7     arm/link3        
arm/link5         8     arm/link4        
arm/link_tip      9     arm/link5        
gripper/base_link 10    arm/link_tip     
gripper/finger1   11    gripper/base_link
gripper/finger2   12    gripper/base_link
gripper/link_tip  13    gripper/base_link
-----------------------------------------
name              index parents          

```
- generate IK solver
```
python `openrave-config --python-dir`/openravepy/_openravepy_/ikfast.py --robot=catchrobo_description/robots/catchrobo.dae --iktype=transform6d --baselink="2" --eelink="8" --savefile="`pwd`/catchrobo_IKFast/ikfast61_arm0.cpp"
``` -->
