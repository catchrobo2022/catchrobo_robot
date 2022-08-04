#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from re import X
import numpy as np
from numpy import False_
from rospy.core import rospyinfo
import rospy

from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Int8


from catchrobo_manager.ros_cmd_template import RosCmdTemplate
from catchrobo_msgs.msg import MyRosCmdArray, MyRosCmd, EnableCmd

# importing original class #the last XBOX~is a class not a module
from joy_controller_class.xbox_button import XBoxButton
from catchrobo_manual.manual_command import ManualCommand

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
        self.button_count = [0] * 12
        self.axes_count = [0] * 6
        # num relate to ps4button_wireless class
        # there are 13+1 list, because need 1~13 not 0~12

        self.zero_flag = 0

        self.manual_cmd = ManualCommand()
        self.manual_msg = Int8()
        self.manual_msg.data = self.manual_cmd.NONE

        self.pause_manual = True
        self.button_enable = True
        self.cmd_flag = [False] * 3

        self.old_joystick = [0.0] * 3
        self.old_button = [0.0] * 3

        self.pub_count = 0

        # Publisher
        self.pub_ros_cmd = rospy.Publisher("ros_cmd", MyRosCmd, queue_size=1)
        self.pub_enable_cmd = rospy.Publisher("enable_cmd", EnableCmd, queue_size=1)

        manual_command_topic = "manual_command"
        self.pub_manual_command = rospy.Publisher(
            manual_command_topic, Int8, queue_size=1
        )

        # Subscriber
        rospy.Subscriber("/joy", Joy, self.joyCallback, queue_size=1)
        rospy.Subscriber("joint_states", JointState, self.jointCallback, queue_size=1)

        from_auto_topic = "auto2manual_command"
        rospy.Subscriber(from_auto_topic, Int8, self.ask_manual_callback, queue_size=1)
        rospy.Subscriber("enable_cmd", EnableCmd, self.enable_callback, queue_size=1)

    def enable_callback(self, msg):
        ### [TODO] 自動制御側からも enableの指示が来る
        pass

    def ask_manual_callback(self, msg):
        ### [TODO] 自動制御側からも manual on offの指示が来るので、それに従う.
        pass

    def joyCallback(self, joy_msg):
        self.joy_state = joy_msg
        self.manualControl()

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
            self.command.command_x.mode = MyRosCmd.VELOCITY_CTRL_MODE
            self.command.command_x.kp = KP_ALL
            self.command.command_x.kd = KD_ALL
        elif axe_num == 1:  # y
            self.command.command_y.mode = MyRosCmd.VELOCITY_CTRL_MODE
            self.command.command_y.kp = KP_ALL
            self.command.command_y.kd = KD_ALL
        elif axe_num == 2:  # z
            self.command.command_z.mode = MyRosCmd.VELOCITY_CTRL_MODE
            self.command.command_z.kp = KP_ALL
            self.command.command_z.kd = KD_ALL
        elif axe_num == 3:  # g
            self.command.command_g.mode = MyRosCmd.POSITION_CTRL_MODE
            self.command.command_g.kp = KP_ALL
            self.command.command_g.kd = KD_ALL

    # 速度制御のパラメーラ設定

    def velSet(self, axe_num):
        KP_ALL = 30.0
        KD_ALL = 0.2
        if axe_num == 0:  # x
            self.command.command_x.mode = MyRosCmd.VELOCITY_CTRL_MODE
            self.command.command_x.kp = KP_ALL
            self.command.command_x.kd = 0.1
        elif axe_num == 1:  # y
            self.command.command_y.mode = MyRosCmd.VELOCITY_CTRL_MODE
            self.command.command_y.kp = 60
            self.command.command_y.kd = KD_ALL
        elif axe_num == 2:  # z
            self.command.command_z.mode = MyRosCmd.VELOCITY_CTRL_MODE
            self.command.command_z.kp = KP_ALL
            self.command.command_z.kd = KD_ALL
        elif axe_num == 3:  # g
            self.command.command_g.mode = MyRosCmd.VELOCITY_CTRL_MODE
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
        m_cmd = self.manual_cmd

        # disable or manual off中は動けない
        if self.button_enable == True:
            if self.pause_manual == False:

                # 速度制御
                # kp0だから、positionは更新しなくていい
                # 速度を一気に調整用
                VELOCITY_VAR = 0.1
                # x軸
                if joy_a[b_num.LX] > 0.3 and self.old_joystick[0] <= 0.3:
                    self.velSet(0)
                    cmd_x.velocity = cmd_tmp.robot_m2rad(
                        cmd_x.id, VELOCITY_VAR * self.COLOR_NUM
                    )
                    self.cmd_flag[0] = True
                    print("left")
                elif joy_a[b_num.LX] < -0.3 and self.old_joystick[0] >= -0.3:
                    self.velSet(0)
                    cmd_x.velocity = cmd_tmp.robot_m2rad(
                        cmd_x.id, (-1) * VELOCITY_VAR * self.COLOR_NUM
                    )
                    self.cmd_flag[0] = True
                    print("right")
                elif (joy_a[b_num.LX] < 0.3 and joy_a[b_num.LX] > -0.3) and (
                    self.old_joystick[0] <= -0.3 or self.old_joystick[0] >= 0.3
                ):
                    cmd_x.velocity = cmd_tmp.robot_m2rad(cmd_x.id, 0.0)
                    self.cmd_flag[0] = True
                    # print("stop1")

                # y軸
                if joy_a[b_num.LY] > 0.3 and self.old_joystick[1] <= 0.3:
                    self.velSet(1)
                    cmd_y.velocity = cmd_tmp.robot_m2rad(
                        cmd_y.id, (-1) * VELOCITY_VAR * self.COLOR_NUM
                    )
                    self.cmd_flag[1] = True
                    # print("forward")
                elif joy_a[b_num.LY] < -0.3 and self.old_joystick[1] >= -0.3:
                    self.velSet(1)
                    cmd_y.velocity = cmd_tmp.robot_m2rad(
                        cmd_y.id, VELOCITY_VAR * self.COLOR_NUM
                    )
                    self.cmd_flag[1] = True
                    # print("back")
                elif (joy_a[b_num.LY] < 0.3 and joy_a[b_num.LY] > -0.3) and (
                    self.old_joystick[1] <= -0.3 or self.old_joystick[1] >= 0.3
                ):
                    cmd_y.velocity = cmd_tmp.robot_m2rad(cmd_y.id, 0.0)
                    self.cmd_flag[1] = True
                    # print("stop2")

                # z軸
                # これは位置制御がいいね。速度制御はいらいらする
                if joy_b[b_num.LB] == 1 and self.old_joystick[2] < 1:
                    self.velSet(2)
                    cmd_z.velocity = cmd_tmp.robot_m2rad(cmd_z.id, VELOCITY_VAR)
                    self.old_joystick[2] = 1
                    self.cmd_flag[2] = True
                    print("up")
                elif joy_b[b_num.LT] == 1 and self.old_joystick[2] > -1:
                    self.velSet(2)
                    cmd_z.velocity = cmd_tmp.robot_m2rad(cmd_z.id, (-1) * VELOCITY_VAR)
                    self.old_joystick[2] = -1
                    self.cmd_flag[2] = True
                    print("down")
                elif joy_b[b_num.LB] == 0 and (
                    self.old_joystick[2] > 0 or self.old_joystick[2] < 0
                ):
                    cmd_z.velocity = cmd_tmp.robot_m2rad(cmd_z.id, 0.0)
                    self.old_joystick[2] = 0
                    self.cmd_flag[2] = True
                elif joy_b[b_num.LT] == 0 and (
                    self.old_joystick[2] > 0 or self.old_joystick[2] < 0
                ):
                    cmd_z.velocity = cmd_tmp.robot_m2rad(cmd_z.id, 0.0)
                    self.old_joystick[2] = 0
                    self.cmd_flag[2] = True

                self.old_joystick[0] = joy_a[b_num.LX]
                self.old_joystick[1] = joy_a[b_num.LY]
                self.old_button[0] = joy_a[b_num.RIGHT_LEFT]
                self.old_button[1] = joy_a[b_num.UP_DOWN]

                ### 指令値適当
                # gripper
                # grab # release
                # ここでpubしてる
                if joy_b[b_num.B] == 1 and self.button_count[b_num.B] == 0:
                    self.posSet(3)
                    cmd_g.position = cmd_tmp.robot_m2rad(cmd_g.id, 1.5)
                    self.pub_ros_cmd.publish(self.command.command_g)
                    self.pubCount("g")
                    self.button_count[b_num.B] = 1
                    print("release")
                elif joy_b[b_num.B] == 0 and self.button_count[b_num.B] == 1:
                    self.button_count[b_num.B] = 2
                elif joy_b[b_num.B] == 1 and self.button_count[b_num.B] == 2:
                    self.posSet(3)
                    cmd_g.position = cmd_tmp.robot_m2rad(cmd_g.id, 0.0)
                    self.pub_ros_cmd.publish(self.command.command_g)
                    self.pubCount("g")
                    self.button_count[b_num.B] = 3
                    print("grab")
                elif joy_b[b_num.B] == 0 and self.button_count[b_num.B] == 3:
                    self.button_count[b_num.B] = 0

                # # 　位置制御
                # # 　一回押すと、一回分動く仕様
                # # 　移動距離が短い場合は台形加速の恩恵がほぼないから、DIRECTで制御にする
                # # x軸
                # if joy_a[b_num.RIGHT_LEFT] == 1 and self.old_button[0] < 1:
                #     self.posSet(0)
                #     self.position_var_manual[0] += cmd_tmp.robot_m2rad(
                #         cmd_x.id, 0.005 * self.COLOR_NUM
                #     )
                #     self.cmd_flag[0] = True
                #     print("b_left")
                # elif joy_a[b_num.RIGHT_LEFT] == -1 and self.old_button[0] > -1:
                #     self.posSet(0)
                #     self.position_var_manual[0] -= cmd_tmp.robot_m2rad(
                #         cmd_x.id, 0.005 * self.COLOR_NUM
                #     )
                #     self.cmd_flag[0] = True
                #     print("b_right")
                # elif joy_a[b_num.RIGHT_LEFT] == 0 and (
                #     self.old_button[0] > 0 or self.old_button[0] < 0
                # ):
                #     self.position_var_manual[0] = 0.0
                #     self.cmd_flag[0] = True

                # # y軸
                # if joy_a[b_num.UP_DOWN] == 1 and self.old_button[1] < 1:
                #     self.posSet(1)
                #     self.position_var_manual[1] -= cmd_tmp.robot_m2rad(
                #         cmd_y.id, 0.005 * self.COLOR_NUM
                #     )
                #     self.cmd_flag[1] = True
                #     print("b_forward")
                # elif joy_a[b_num.UP_DOWN] == -1 and self.old_button[1] > -1:
                #     self.posSet(1)
                #     self.position_var_manual[1] += cmd_tmp.robot_m2rad(
                #         cmd_y.id, 0.005 * self.COLOR_NUM
                #     )
                #     self.cmd_flag[1] = True
                #     print("b_back")
                # elif joy_a[b_num.UP_DOWN] == 0 and (
                #     self.old_button[1] > 0 or self.old_button[1] < 0
                # ):
                #     self.position_var_manual[1] = 0.0
                #     self.cmd_flag[1] = True

                ### manual if

            ### 自動→手動切り替え # manual offはpauseを兼ねる
            # manual off
            if joy_b[b_num.A] == 1:
                self.manual_msg.data = m_cmd.MANUAL_OFF
                self.pause_manual = True
                self.pauseManual()
                self.pub_manual_command.publish(self.manual_msg)
                self.button_count[b_num.A] = 1
                print("b_manual_off")
            # manual on
            if joy_b[b_num.Y] == 1:
                self.manual_msg.data = m_cmd.MANUAL_ON
                self.pause_manual = False
                self.pub_manual_command.publish(self.manual_msg)
                print("b_manual_on")

            # # 待機　pause
            # # 入力を受け付けない感じ#positionに現在位置を入れ続ける。
            # # 一回目押したとき、pauseを維持
            # if joy_b[b_num.A] == 1 and (
            #     self.button_count[b_num.A] == 0 or self.button_count[b_num.A] == 1
            # ):
            #     # 　本来はここにはいらないけど、安全のために一応入れとく
            #     self.pauseProcess()
            #     self.button_count[b_num.A] = 1
            #     # print("pause start")
            # elif joy_b[b_num.A] == 0 and (
            #     self.button_count[b_num.A] == 1 or self.button_count[b_num.A] == 2
            # ):
            #     self.pauseProcess()
            #     self.button_count[b_num.A] = 2
            #     print("pause")
            # # 二回目押したとき、pause機能を停止
            # elif joy_b[b_num.A] == 1 and self.button_count[b_num.A] == 2:
            #     # これ以降の分岐はXの分岐の方の基本の方でposが入力される
            #     self.button_count[b_num.A] = 3
            #     # print("restart start")
            # elif joy_b[b_num.A] == 0 and self.button_count[b_num.A] == 3:
            #     self.button_count[b_num.A] = 0
            #     self.is_pause = False
            #     print("restart")

            ### enable if

        # is_enableのon,offの処理 # ボタンを押すとon, off 切り替わる
        if joy_b[b_num.START] == 1 and self.button_count[b_num.START] == 0:
            self.command.enable_command.is_enable = False
            # print(self.command.enable_command)
            self.button_count[b_num.START] = 1
            print("disable")
            # print(self.button_count[b_num.START])
            self.button_enable = False
        elif joy_b[b_num.START] == 0 and self.button_count[b_num.START] == 1:
            self.pub_enable_cmd.publish(self.command.enable_command)
            self.button_count[b_num.START] = 2
        elif joy_b[b_num.START] == 1 and self.button_count[b_num.START] == 2:
            self.command.enable_command.is_enable = True
            # print(self.command.enable_command)
            self.button_count[b_num.START] = 3
            print("enable")
            self.button_enable = True
        elif joy_b[b_num.START] == 0 and self.button_count[b_num.START] == 3:
            self.pub_enable_cmd.publish(self.command.enable_command)
            self.button_count[b_num.START] = 0

    # pauseのときの処理
    # gripperだけ止めない
    # def pauseProcess(self):
    #     cmd_x = self.command.command_x
    #     cmd_y = self.command.command_y
    #     cmd_z = self.command.command_z
    #     cmd_tmp = self.command.template

    #     self.is_pause = True

    #     # 位置　現在位置を保持
    #     cmd_x.position = cmd_tmp.robot_m2rad(
    #         cmd_x.id, self.joint_current_pos[0])
    #     cmd_y.position = cmd_tmp.robot_m2rad(
    #         cmd_y.id, self.joint_current_pos[1])
    #     cmd_z.position = cmd_tmp.robot_m2rad(
    #         cmd_z.id, self.joint_current_pos[2])
    #     # 速度　すべて初期値
    #     cmd_x.velocity = cmd_tmp.robot_m2rad(cmd_x.id, 0.0)
    #     cmd_y.velocity = cmd_tmp.robot_m2rad(cmd_y.id, 0.0)
    #     cmd_z.velocity = cmd_tmp.robot_m2rad(cmd_z.id, 0.0)

    #     self.pub_ros_cmd.publish(self.command.command_x)
    #     self.pubCount("pX")
    #     self.pub_ros_cmd.publish(self.command.command_y)
    #     self.pubCount("pY")
    #     self.pub_ros_cmd.publish(self.command.command_z)
    #     self.pubCount("pZ")

    def pauseManual(self):
        ### [TODO] bugが出たので、とりあえず消しています
        return

        cmd_x = self.command.command_x
        cmd_y = self.command.command_y
        cmd_z = self.command.command_z
        cmd_tmp = self.command.template

        # 速度　すべて初期値
        cmd_x.velocity = cmd_tmp.robot_m2rad(cmd_x.id, 0.0)
        cmd_y.velocity = cmd_tmp.robot_m2rad(cmd_y.id, 0.0)
        cmd_z.velocity = cmd_tmp.robot_m2rad(cmd_z.id, 0.0)

        self.pub_ros_cmd.publish(self.command.command_x)
        self.pubCount("pX")
        self.pub_ros_cmd.publish(self.command.command_y)
        self.pubCount("pY")
        self.pub_ros_cmd.publish(self.command.command_z)
        self.pubCount("pZ")

    # def showButtonRole(self):
    #     print('''
    #     left joystick : vel xy
    #     LB : vel z+
    #     LT ; vel z-

    #     horizontal button : pos x
    #     vertical button : pos y

    #     B : gripper grab & release
    #     A : pause

    #     back: manual on
    #     start: enable

    #     ''')

    def pubXYZ(self, axes):
        cmd_x = self.command.command_x
        cmd_y = self.command.command_y
        cmd_z = self.command.command_z
        cmd_tmp = self.command.template
        if self.pause_manual == False:
            if axes == "x":
                cmd_x.position = self.position_var_manual[0] + cmd_tmp.robot_m2rad(
                    cmd_x.id, self.joint_current_pos[0]
                )
                # print("pos_var",self.position_var_manual[0])
                # print("cmd_pos",cmd_x.position)
                self.pub_ros_cmd.publish(self.command.command_x)
                self.pubCount("x")
            elif axes == "y":
                cmd_y.position = self.position_var_manual[1] + cmd_tmp.robot_m2rad(
                    cmd_y.id, self.joint_current_pos[1]
                )
                self.pub_ros_cmd.publish(self.command.command_y)
                self.pubCount("y")
            elif axes == "z":
                cmd_z.position = self.position_var_manual[2] + cmd_tmp.robot_m2rad(
                    cmd_z.id, self.joint_current_pos[2]
                )
                self.pub_ros_cmd.publish(self.command.command_z)
                self.pubCount("z")

    def pubCount(self, axes):
        self.pub_count += 1
        print("count", axes, self.pub_count)

    def main(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            if self.cmd_flag[0] == True:
                self.pubXYZ("x")
                self.cmd_flag[0] = False
            if self.cmd_flag[1] == True:
                self.pubXYZ("y")
                self.cmd_flag[1] = False
            if self.cmd_flag[2] == True:
                self.pubXYZ("z")
                self.cmd_flag[2] = False
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("manual_xbox_node", anonymous=True)
    rospy.loginfo("Manual Mode!!")
    manual = Manual(XBoxButton)
    manual.main()
