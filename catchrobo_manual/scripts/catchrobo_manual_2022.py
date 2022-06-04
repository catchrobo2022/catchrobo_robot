#!/usr/bin/env python
# -*- coding: utf-8 -*-


import numpy as np
from rospy.core import rospyinfo
import rospy

from sensor_msgs.msg import Joy

from catchrobo_msgs.msg import MyRosCmdArray, MyRosCmd

class PS4Button_wireless:
    # Axes
    LX = 0 #horizontal
    LY = 1 #vertical
    RX = 2
    RY = 3
    UP_DOWN = 10
    RIGHT_LEFT=9
    # Buttons
    L3 = 10
    R3 = 11
    L2=6
    R2=7
    L1=4
    R1=5
    TRIANGLE=3
    CIRCLE=2
    CROSS=1
    SQUARE=0


class PS4Button: #　有線のとき
    # Axes
    LX = 0  # horizontal
    LY = 1  # vertical
    RX = 2
    RY = 3
    UP_DOWN = 7
    RIGHT_LEFT = 6
    # Buttons
    L3 = 10
    R3 = 11
    L2 = 6
    R2 = 7
    L1 = 4
    R1 = 5
    TRIANGLE = 3
    CIRCLE = 2
    CROSS = 1
    SQUARE = 0


# mbed_sim_test,pyの中身を移植したクラス
class Command():
    def __init__(self):
        self.command_x = MyRosCmd()  # MyRosCmd型の自作msg型のインスタンス
        self.command_y = MyRosCmd()
        self.command_z = MyRosCmd()
        self.command_g = MyRosCmd() # gripperのg
        self.command_array = MyRosCmdArray() #MyRosCmdArray型の自作msg型のインスタンス
        self.mannualInput()

    #設定したやつを出力用変数に突っ込む
    def setCommandArray(self):
        self.command_array.command_array = [self.command_x, self.command_y, self.command_z, self.command_g]

    def getCommandX(self):
        return self.command_x
    
    def getCommandY(self):
        return self.command_y

    def getCommandZ(self):
        return self.command_z

    def getCommandG(self):
        return self.command_g
    
    #　手動で値をいじりたいとき用
    #  初期化で値をいじる（引数に入れてつかわないから変更には使えない）
    #　position ,velocityだけの変更なら関数いらん
    def mannualInput(self):
        print("input")
        
        position_var=[0,0,0,0]
        velocity_var=[0,0,0,0]
        # 手動設定用
        # position_var = [1, 1, 0, 0]
        # velocity_var = [0, 0, 0, 0]

        #位置制御
        self.command_x.id = 0    # 0: x軸
        self.command_x.mode = MyRosCmd.POSITION_CTRL_MODE  # MyRosCmd.POSITION_CTRL_MODE or MyRosCmd.VELOCITY_CTRL_MODE
        self.command_x.position = position_var[0]
        self.command_x.velocity = velocity_var[0]
        self.command_x.inertia = 1.0
        self.command_x.effort = 0
        self.command_x.position_min = 0
        self.command_x.position_max = 1.5
        self.command_x.velocity_limit = 1
        self.command_x.acceleration_limit = 1
        self.command_x.jerk_limit = 1
        self.command_x.kp = 0
        self.command_x.kd = 1 

        self.command_y.id = 1   # 1:y軸 
        self.command_y.mode = MyRosCmd.POSITION_CTRL_MODE  # MyRosCmd.POSITION_CTRL_MODE or MyRosCmd.VELOCITY_CTRL_MODE
        self.command_y.position = position_var[1]
        self.command_y.velocity = velocity_var[1]
        self.command_y.inertia = 1.0
        self.command_y.effort = 0
        self.command_y.position_min = -0.8
        self.command_y.position_max = 0.8
        self.command_y.velocity_limit = 2
        self.command_y.acceleration_limit = 2
        self.command_y.jerk_limit = 2
        self.command_y.kp = 0
        self.command_y.kd = 1 

        self.command_z.id = 2    # 2: z軸
        self.command_z.mode = MyRosCmd.POSITION_CTRL_MODE  # MyRosCmd.POSITION_CTRL_MODE or MyRosCmd.VELOCITY_CTRL_MODE
        self.command_z.position = position_var[2]
        self.command_z.velocity = velocity_var[2]
        self.command_z.inertia = 1.0
        self.command_z.effort = 0
        self.command_z.position_min = 0
        self.command_z.position_max = 0.5
        self.command_z.velocity_limit = 1
        self.command_z.acceleration_limit = 1
        self.command_z.jerk_limit = 1
        self.command_z.kp = 0
        self.command_z.kd = 1 

        self.command_g.id = 3    #3:グリッパー
        self.command_g.mode = MyRosCmd.POSITION_CTRL_MODE  # MyRosCmd.POSITION_CTRL_MODE or MyRosCmd.VELOCITY_CTRL_MODE
        self.command_g.position = position_var[3]
        self.command_g.velocity = velocity_var[3]
        self.command_g.inertia = 1.0
        self.command_g.effort = 0
        self.command_g.position_min = 0
        self.command_g.position_max = 1.5
        self.command_g.velocity_limit = 2
        self.command_g.acceleration_limit = 2
        self.command_g.jerk_limit = 2
        self.command_g.kp = 0
        self.command_g.kd = 1

        self.setCommandArray() 
    
class Manual():
    def __init__(self,Button):
        self._color = rospy.get_param("/color", default="red") #　つかえるのかわからん　#なんだこれ #　とりあえずフィールドは赤を想定

        self.command = Command()  # Commandクラスのインスタンス
        self.joy_state = Joy()  # joy型のmsgのインスタンス
        self.ButtonEnum = Button
        self.position_var_manual = [0,0,0,0]
        self.button_count = [0, 0, 0, 0]

        self.my_joint_control_pub = rospy.Publisher("/my_joint_control", MyRosCmdArray, queue_size=1)
        rospy.Subscriber("joy", Joy, self.joyCallback, queue_size=1)

    def joyCallback(self, joy_msg):
        self.joy_state = joy_msg
        self.my_joint_control_pub.publish(self.command.command_array)
        self.manualControl()
        
    def getMsg(self):
        return self.joy_state

    #　ここでボタンによる操作のせていをしてる
    def manualControl(self):
        #速度制御
        # x軸
        if(self.joy_state.axes[self.ButtonEnum.LX] > 0.3):
            self.command.command_x.mode = MyRosCmd.VELOCITY_CTRL_MODE
            self.command.command_x.velocity=1
            # print("left")
        elif(self.joy_state.axes[self.ButtonEnum.LX] < -0.3):
            self.command.command_x.mode = MyRosCmd.VELOCITY_CTRL_MODE
            self.command.command_x.velocity = -1
            # print("right")
        else:
            self.command.command_x.velocity=0
            # print("stop1")
        
        #y軸
        if(self.joy_state.axes[self.ButtonEnum.LY] > 0.3):
            self.command.command_y.mode = MyRosCmd.VELOCITY_CTRL_MODE
            self.command.command_y.velocity = -2
            # print("forward")
        elif(self.joy_state.axes[self.ButtonEnum.LY] < -0.3):
            self.command.command_y.mode = MyRosCmd.VELOCITY_CTRL_MODE
            self.command.command_y.velocity = 2
            # print("back")
        else:
            self.command.command_y.velocity = 0
            # print("stop2")

        #z軸
        #これは位置制御がいいね。速度制御はいらいらする
        if(self.joy_state.buttons[self.ButtonEnum.L1] == 1):
            self.command.command_z.mode = MyRosCmd.VELOCITY_CTRL_MODE
            self.command.command_z.velocity = 1
            # print("up")
        elif(self.joy_state.buttons[self.ButtonEnum.L2] == 1):
            self.command.command_z.mode = MyRosCmd.VELOCITY_CTRL_MODE
            self.command.command_z.velocity = -1
            # print("down")
        else:
            self.command.command_z.velocity = 0
            # print("stop3")
        
        #　位置制御
        #x軸
        if(self.joy_state.axes[self.ButtonEnum.RIGHT_LEFT] == 1 and self.button_count[0]==0):
            self.command.command_x.mode = MyRosCmd.POSITION_CTRL_MODE
            self.position_var_manual[0]+=0.1
            
            self.button_count[0]=1
            print("b_left")
        elif(self.joy_state.axes[self.ButtonEnum.RIGHT_LEFT] == -1 and self.button_count[0]==0):
            self.command.command_x.mode = MyRosCmd.POSITION_CTRL_MODE
            self.position_var_manual[0]-=0.1
            self.button_count[0] = 1
            print("b_right")
        # elif(self.joy_state.axes[self.ButtonEnum.RIGHT_LEFT] == 1 and self.button_count[0] == 1):
        #     self.command.command_x.velocity = 1
        # elif(self.joy_state.axes[self.ButtonEnum.RIGHT_LEFT] == -1 and self.button_count[0] == 1):
        #     self.command.command_x.velocity = -1
        elif(self.joy_state.axes[self.ButtonEnum.RIGHT_LEFT] == 0):
            self.button_count[0] = 0
            

        self.command.command_x.position = self.position_var_manual[0]
        
        #y軸
        if(self.joy_state.axes[self.ButtonEnum.UP_DOWN] == 1 and self.button_count[1] == 0):
            self.command.command_x.mode = MyRosCmd.POSITION_CTRL_MODE
            self.position_var_manual[1] += 0.1
            self.button_count[1] = 1
            print("b_forward")
        elif(self.joy_state.axes[self.ButtonEnum.UP_DOWN] == -1 and self.button_count[1] == 0):
            self.command.command_x.mode = MyRosCmd.POSITION_CTRL_MODE
            self.position_var_manual[1] -= 0.1
            self.button_count[1] = 1
            print("b_back")
        elif(self.joy_state.axes[self.ButtonEnum.UP_DOWN] == 0):
            self.button_count[1]= 0

        self.command.command_y.position = self.position_var_manual[1]

        #gripper
        if(self.joy_state.buttons[self.ButtonEnum.CIRCLE] == 1):
            self.command.command_x.mode = MyRosCmd.POSITION_CTRL_MODE
            self.position_var_manual[3] =1.0
            print("b_circle")
        elif(self.joy_state.buttons[self.ButtonEnum.CROSS] == 1):
            self.command.command_x.mode = MyRosCmd.POSITION_CTRL_MODE
            self.position_var_manual[3] =0
            print("b_cross")

        self.command.command_g.position = self.position_var_manual[3]


        self.command.setCommandArray()

    def main(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            #print(vars(self.command))
            # print("p_var=", self.position_var_manual[0])
            print(self.command.getCommandY())
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("ManualManager", anonymous=True)
    rospy.loginfo("Manual Mode!!")
    manual = Manual(PS4Button_wireless)
    manual.main()


    

    
   
