#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
from rospy.core import rospyinfo
import rospy

from sensor_msgs.msg import Joy
from sensor_msgs.msg import JointState
from std_msgs.msg import Int16

from catchrobo_driver.ros_cmd_template import RosCmdTemplate
from catchrobo_msgs.msg import MyRosCmdArray, MyRosCmd, EnableCmd

# importing original class #the last XBOX~is a class not a module
from joy_controller_class.xbox_button import XBoxButton


# mbed_sim_test,pyの中身を移植したクラス
class Command:
    def __init__(self):
        self.template = RosCmdTemplate()
        self.enable_command = self.template.generate_enable_command()

        # 　position ,velocityの初期値はをここでいじる
        self.command_x = self.template.generate_ros_command(
            id=0,
            mode=MyRosCmd.POSITION_CTRL_MODE,
            robot_position=0.0,
            robot_end_velocity=0.0,
        )
        self.command_y = self.template.generate_ros_command(
            id=1,
            mode=MyRosCmd.POSITION_CTRL_MODE,
            robot_position=0.0,
            robot_end_velocity=0.0,
        )
        self.command_z = self.template.generate_ros_command(
            id=2,
            mode=MyRosCmd.POSITION_CTRL_MODE,
            robot_position=0.0,
            robot_end_velocity=0.0,
        )
        self.command_g = self.template.generate_ros_command(
            id=3,
            mode=MyRosCmd.POSITION_CTRL_MODE,
            robot_position=0.0,
            robot_end_velocity=0.0,
        )  # gripperのg

        self.command_array = MyRosCmdArray()  # MyRosCmdArray型の自作msg型のインスタンス
        self.manualInput()

    def getCommandX(self):
        return self.command_x

    def getCommandY(self):
        return self.command_y

    def getCommandZ(self):
        return self.command_z

    def getCommandG(self):
        return self.command_g

    # 　手動で値をいじりたいとき用
    #  初期化で値をいじる（引数に入れてつかわないから変更には使えない）
    # 　position ,velocityだけの変更なら関数いらん
    # 　いらない
    def manualInput(self):
        print("input")
        print("フィールド全部試す")


class Manual:
    def __init__(self, Button):
        # launchのparam取得
        # private paramというやつらしい#おそらくnodeタグの中だからだと思われる
        self.field_color = rospy.get_param("/field")
        self.COLOR_NUM = 1.0
        if self.field_color == "red":
            # これ実はいらないけど、まあわかりやすい用
            self.COLOR_NUM = 1.0
            print("field red")
        elif self.field_color == "blue":
            self.COLOR_NUM = -1.0
            print("field blue")

        self.command = Command()  # Commandクラスのインスタンス
        self.joy_state = Joy()  # joy型のmsgのインスタンス
        self.ButtonEnum = Button
        self.joint_current_pos = [0.0] * 4
        self.position_var_manual = [0.0] * 4
        self.button_count = [0] * 14
        self.axes_count = [0] * 11
        # num relate to ps4button_wireless class
        # there are 13+1 list, because need 1~13 not 0~12

        self.zero_flag = 0
        self.enable_manual = 0
        self.button_disable = False

        # 一つだけの軸で指令を送りたい場合用
        self.pub_ros_cmd = rospy.Publisher("/ros_cmd", MyRosCmd, queue_size=1)
        self.pub_enable_cmd = rospy.Publisher("/enable_cmd", EnableCmd, queue_size=1)
        self.pub_manual_mode = rospy.Publisher(
            "/enable_manual_mode", Int16, queue_size=1
        )

        rospy.Subscriber("/joy", Joy, self.joyCallback, queue_size=1)
        rospy.Subscriber("/joint_states", JointState, self.jointCallback, queue_size=1)

    def joyCallback(self, joy_msg):
        self.joy_state = joy_msg
        self.manualControl()

        self.pub_manual_mode.publish(self.enable_manual)

        # 一つだけの軸で指令を送りたい場合用
        self.pub_ros_cmd.publish(self.command.command_x)
        self.pub_ros_cmd.publish(self.command.command_y)
        self.pub_ros_cmd.publish(self.command.command_z)
        # gripperだけ押したときにpub

    # 　現在位置取得用

    def jointCallback(self, joint_state_msg):
        self.joint_state_feedback = joint_state_msg
        if self.zero_flag == 0:
            for i in range(4):
                self.joint_current_pos[i] = self.joint_state_feedback.position[i]
            self.joint_current_pos[1] *= 2

    def getMsg(self):
        return self.joy_state

    # 位置制御のパラメーラ設定
    def posSet(self, axe_num):
        KP_ALL = 5.0
        KD_ALL = 0.05
        if axe_num == 0:  # x
            self.command.command_x.mode = MyRosCmd.DIRECT_CTRL_MODE
            self.command.command_x.kp = KP_ALL
            self.command.command_x.kd = KD_ALL
        elif axe_num == 1:  # y
            self.command.command_y.mode = MyRosCmd.DIRECT_CTRL_MODE
            self.command.command_y.kp = KP_ALL
            self.command.command_y.kd = KD_ALL
        elif axe_num == 2:  # z
            self.command.command_z.mode = MyRosCmd.DIRECT_CTRL_MODE
            self.command.command_z.kp = KP_ALL
            self.command.command_z.kd = KD_ALL
        elif axe_num == 3:  # g
            self.command.command_g.mode = MyRosCmd.POSITION_CTRL_MODE
            self.command.command_g.kp = KP_ALL
            self.command.command_g.kd = KD_ALL

    # 速度制御のパラメーラ設定

    def velSet(self, axe_num):
        KP_ALL = 0.0
        KD_ALL = 0.5
        if axe_num == 0:  # x
            self.command.command_x.mode = MyRosCmd.DIRECT_CTRL_MODE
            self.command.command_x.kp = KP_ALL
            self.command.command_x.kd = KD_ALL
        elif axe_num == 1:  # y
            self.command.command_y.mode = MyRosCmd.DIRECT_CTRL_MODE
            self.command.command_y.kp = KP_ALL
            self.command.command_y.kd = KD_ALL
        elif axe_num == 2:  # z
            self.command.command_z.mode = MyRosCmd.DIRECT_CTRL_MODE
            self.command.command_z.kp = KP_ALL
            self.command.command_z.kd = KD_ALL
        elif axe_num == 3:  # g
            self.command.command_g.mode = MyRosCmd.DIRECT_CTRL_MODE
            self.command.command_g.kp = KP_ALL
            self.command.command_g.kd = KD_ALL

    # 　ここでボタンによる操作のせていをしてる
    def manualControl(self):
        cmd_x = self.command.command_x
        cmd_y = self.command.command_y
        cmd_z = self.command.command_z
        cmd_g = self.command.command_g
        joy_a = self.joy_state.axes
        joy_b = self.joy_state.buttons
        b_num = self.ButtonEnum
        cmd_tmp = self.command.template

        # disable中は動けない
        if self.button_disable == False:
            # 速度制御
            # kp0だから、positionは更新しなくていい
            # 速度を一気に調整用
            VELOCITY_VAR = 0.1
            # x軸
            if joy_a[b_num.LX] > 0.3:
                self.velSet(0)
                cmd_x.velocity = cmd_tmp.robot_m2rad(
                    cmd_x.id, VELOCITY_VAR * self.COLOR_NUM
                )
                # print("left")
            elif joy_a[b_num.LX] < -0.3:
                self.velSet(0)
                cmd_x.velocity = cmd_tmp.robot_m2rad(
                    cmd_x.id, (-1) * VELOCITY_VAR * self.COLOR_NUM
                )
                # print("right")
            else:
                cmd_x.velocity = cmd_tmp.robot_m2rad(cmd_x.id, 0.0)
                # print("stop1")

            # y軸
            if joy_a[b_num.LY] > 0.3:
                self.velSet(1)
                cmd_y.velocity = cmd_tmp.robot_m2rad(
                    cmd_y.id, (-1) * VELOCITY_VAR * self.COLOR_NUM
                )
                # print("forward")
            elif joy_a[b_num.LY] < -0.3:
                self.velSet(1)
                cmd_y.velocity = cmd_tmp.robot_m2rad(
                    cmd_y.id, VELOCITY_VAR * self.COLOR_NUM
                )
                # print("back")
            else:
                cmd_y.velocity = cmd_tmp.robot_m2rad(cmd_y.id, 0.0)
                # print("stop2")

            # z軸
            # これは位置制御がいいね。速度制御はいらいらする
            if joy_b[b_num.LB] == 1:
                self.velSet(2)
                cmd_z.velocity = cmd_tmp.robot_m2rad(cmd_z.id, VELOCITY_VAR)
                print("up")
            elif joy_a[b_num.LT] == -1:
                self.velSet(2)
                cmd_z.velocity = cmd_tmp.robot_m2rad(cmd_z.id, (-1) * VELOCITY_VAR)
                print("down")
            else:
                cmd_z.velocity = cmd_tmp.robot_m2rad(cmd_z.id, 0.0)
                # print("stop3")

            # 　位置制御
            # 　一回押すと、一回分動く仕様
            # 　移動距離が短い場合は台形加速の恩恵がほぼないから、DIRECTで制御にする
            # x軸
            if joy_a[b_num.RIGHT_LEFT] == 1:
                self.posSet(0)
                self.position_var_manual[0] += cmd_tmp.robot_m2rad(
                    cmd_x.id, 0.005 * self.COLOR_NUM
                )
                print("b_left")
            elif joy_a[b_num.RIGHT_LEFT] == -1:
                self.posSet(0)
                self.position_var_manual[0] -= cmd_tmp.robot_m2rad(
                    cmd_x.id, 0.005 * self.COLOR_NUM
                )
                print("b_right")
            elif joy_a[b_num.RIGHT_LEFT] == 0:
                self.position_var_manual[0] = 0.0

            # y軸
            if joy_a[b_num.UP_DOWN] == 1:
                self.posSet(1)
                self.position_var_manual[1] -= cmd_tmp.robot_m2rad(
                    cmd_y.id, 0.005 * self.COLOR_NUM
                )
                print("b_forward")
            elif joy_a[b_num.UP_DOWN] == -1:
                self.posSet(1)
                self.position_var_manual[1] += cmd_tmp.robot_m2rad(
                    cmd_y.id, 0.005 * self.COLOR_NUM
                )
                print("b_back")
            elif joy_a[b_num.UP_DOWN] == 0:
                self.position_var_manual[1] = 0.0

            ### 指令値適当
            # gripper
            # grab # release
            # ここでpubしてる
            if joy_b[b_num.B] == 1 and self.button_count[b_num.B] == 0:
                self.posSet(3)
                self.position_var_manual[3] = cmd_tmp.robot_m2rad(cmd_g.id, 1.5)
                self.button_count[b_num.B] = 1
                print("release")
            elif joy_b[b_num.B] == 0 and self.button_count[b_num.B] == 1:
                self.button_count[b_num.B] = 2
                self.pub_ros_cmd.publish(self.command.command_g)
            elif joy_b[b_num.B] == 1 and self.button_count[b_num.B] == 2:
                self.posSet(3)
                self.position_var_manual[3] = cmd_tmp.robot_m2rad(cmd_g.id, 0.0)
                self.button_count[b_num.B] = 3
                print("grab")
            elif joy_b[b_num.B] == 0 and self.button_count[b_num.B] == 3:
                self.button_count[b_num.B] = 0
                self.pub_ros_cmd.publish(self.command.command_g)

            # # いらない
            # # 原点にもどす
            # # 長押ししてね
            # # よくわからないけど、しきい値がある　29ぐらい 0.5mぐらいの位置？ なおった？
            # if(joy_b[b_num.X] == 1):
            #     #原点に戻す処理
            #     print("zero")
            #     for i in range(4):
            #         self.posSet(i)
            #     cmd_x.position = cmd_tmp.robot_m2rad(cmd_x.id, 0.01)
            #     cmd_y.position = cmd_tmp.robot_m2rad(cmd_y.id, 0.0)
            #     cmd_z.position = cmd_tmp.robot_m2rad(cmd_z.id, 0.01)
            #     cmd_g.position = cmd_tmp.robot_m2rad(cmd_g.id, 0.0)
            #     self.zero_flag = 1
            # elif(joy_b[b_num.X] == 0):
            #     #ここが基本のところ
            #     self.zero_flag = 0
            #     cmd_x.position = self.position_var_manual[0] + cmd_tmp.robot_m2rad(
            #         cmd_x.id, self.joint_current_pos[0])
            #     cmd_y.position = self.position_var_manual[1] + cmd_tmp.robot_m2rad(
            #         cmd_y.id, self.joint_current_pos[1])
            #     cmd_z.position = self.position_var_manual[2] + \
            #         cmd_tmp.robot_m2rad(cmd_z.id, self.joint_current_pos[2])
            #     cmd_g.position = self.position_var_manual[3]

            cmd_x.position = self.position_var_manual[0] + cmd_tmp.robot_m2rad(
                cmd_x.id, self.joint_current_pos[0]
            )
            cmd_y.position = self.position_var_manual[1] + cmd_tmp.robot_m2rad(
                cmd_y.id, self.joint_current_pos[1]
            )
            cmd_z.position = self.position_var_manual[2] + cmd_tmp.robot_m2rad(
                cmd_z.id, self.joint_current_pos[2]
            )
            cmd_g.position = self.position_var_manual[3]

        # 自動→手動切り替え
        if joy_b[b_num.XBOX] == 1:
            self.enable_manual = 1
            print("b_ps")

        # # menu
        # if(joy_b[b_num.TOUCH_PAD] == 1 and self.button_count[b_num.TOUCH_PAD] == 0):
        #     self.showButtonRole()
        #     self.button_count[b_num.TOUCH_PAD] = 1
        # elif(joy_b[b_num.TOUCH_PAD] == 0):
        #     self.button_count[b_num.TOUCH_PAD] = 0

        # 待機　pause
        # 入力を受け付けない感じ#positionに現在位置を入れ続ける。
        # 一回目押したとき、pauseを維持
        if joy_b[b_num.A] == 1 and (
            self.button_count[b_num.A] == 0 or self.button_count[b_num.A] == 1
        ):
            # 　本来はここにはいらないけど、安全のために一応入れとく
            self.pauseProcess()
            self.button_count[b_num.A] = 1
            # print("pause start")
        elif joy_b[b_num.A] == 0 and (
            self.button_count[b_num.A] == 1 or self.button_count[b_num.A] == 2
        ):
            self.pauseProcess()
            self.button_count[b_num.A] = 2
            print("pause")
        # 二回目押したとき、pause機能を停止
        elif joy_b[b_num.A] == 1 and self.button_count[b_num.A] == 2:
            # これ以降の分岐はXの分岐の方の基本の方でposが入力される
            self.button_count[b_num.A] = 3
            # print("restart start")
        elif joy_b[b_num.A] == 0 and self.button_count[b_num.A] == 3:
            self.button_count[b_num.A] = 0
            print("restart")

        # is_enableのon,offの処理 # ボタンを押すとon, off 切り替わる
        if joy_b[b_num.START] == 1 and self.button_count[b_num.START] == 0:
            self.command.enable_command.is_enable = True
            # print(self.command.enable_command)
            self.button_count[b_num.START] = 1
            print("enable")
            # print(self.button_count[b_num.START])
            self.button_disable = False
        elif joy_b[b_num.START] == 0 and self.button_count[b_num.START] == 1:
            self.pub_enable_cmd.publish(self.command.enable_command)
            self.button_count[b_num.START] = 2
        elif joy_b[b_num.START] == 1 and self.button_count[b_num.START] == 2:
            self.command.enable_command.is_enable = False
            # print(self.command.enable_command)
            self.button_count[b_num.START] = 3
            print("disable")
            self.button_disable = True
        elif joy_b[b_num.START] == 0 and self.button_count[b_num.START] == 3:
            self.pub_enable_cmd.publish(self.command.enable_command)
            self.button_count[b_num.START] = 0

    # pauseのときの処理
    # gripperだけ止めない
    def pauseProcess(self):
        cmd_x = self.command.command_x
        cmd_y = self.command.command_y
        cmd_z = self.command.command_z
        cmd_tmp = self.command.template

        # 位置　現在位置を保持
        cmd_x.position = cmd_tmp.robot_m2rad(cmd_x.id, self.joint_current_pos[0])
        cmd_y.position = cmd_tmp.robot_m2rad(cmd_y.id, self.joint_current_pos[1])
        cmd_z.position = cmd_tmp.robot_m2rad(cmd_z.id, self.joint_current_pos[2])
        # 速度　すべて初期値
        cmd_x.velocity = cmd_tmp.robot_m2rad(cmd_x.id, 0.0)
        cmd_y.velocity = cmd_tmp.robot_m2rad(cmd_y.id, 0.0)
        cmd_z.velocity = cmd_tmp.robot_m2rad(cmd_z.id, 0.0)

    def showButtonRole(self):
        print(
            """
        left joystick : vel xy
        LB : vel z+
        LT ; vel z-

        horizontal button : pos x
        vertical button : pos y

        B : gripper grab & release
        A : pause
        X : zero point

        xbox: manual on
        start: enable

        touch_pad : show button role

        """
        )

    def main(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            # print(vars(self.command))
            # print("p_var=", self.position_var_manual[0])
            # print(self.command.getCommandX())
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("manual_xbox_node", anonymous=True)
    rospy.loginfo("Manual Mode!!")
    manual = Manual(XBoxButton)
    manual.main()
