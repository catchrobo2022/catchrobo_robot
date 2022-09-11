#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from pickle import PickleError
from catchrobo_manager.gui_menu_enum import GuiMenu
from catchrobo_manager.jagarico.database import Database
from catchrobo_manual.manual_command import ManualCommand
import rospy
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Int8
from geometry_msgs.msg import PoseStamped

import numpy as np


class PointLog:
    def __init__(self):
        auto2gui = "gl_giro"
        gui2auto = "gl_rigo"
        gui_menu = "menu"
        self.FIELD = rospy.get_param("field")
        position_topic = rospy.get_param("radius2meter/world_position_topic")

        self._main_start = False

        rospy.Subscriber("manual_command", Int8, self.manual_callback)
        rospy.Subscriber(position_topic, PoseStamped, self.pose_callback, queue_size=1)
        rospy.Subscriber(
            auto2gui, Int32MultiArray, self.auto2gui_callback, queue_size=1
        )
        rospy.Subscriber(
            gui2auto, Int32MultiArray, self.gui2auto_callback, queue_size=1
        )
        rospy.Subscriber(gui_menu, Int8, self.gui_callback)

    #############################################################
    ### callback
    #############################################################

    def game_start(self):
        # rospy.loginfo("Game start")
        if self._main_start is True:
            return
        top = "calibrated/{}".format(self.FIELD)
        self._database = Database()
        self._database.readCsv(top + "_shoot.csv")
        self._database.add_column("picked_x", 0)
        self._database.add_column("picked_y", 0)
        self._database.add_column("diff_x", 0)
        self._database.add_column("diff_y", 0)
        self._old_data = self._database.getColumn("exist")
        self._main_start = True

    def manual_callback(self, msg):
        if msg.data == ManualCommand.START:
            self.game_start()

    def gui_callback(self, msg):
        if msg.data == GuiMenu.START:
            self.game_start()

    def pose_callback(self, msg: PoseStamped) -> None:
        self._robot_pose = msg

    def gui2auto_callback(self, msg):
        self._old_data = np.asarray(msg.data)

    def auto2gui_callback(self, msg):
        if self._main_start is False:
            return
        # rospy.loginfo("shot")
        data = np.asarray(msg.data)
        diff = data - self._old_data
        # rospy.loginfo(diff)
        self._old_data = data
        if np.sum(diff) != 1:
            return
        pick_id = np.where(diff == 1)[0][0]
        # rospy.loginfo("pick_id")
        # rospy.loginfo(pick_id)
        current_pose = self._robot_pose
        ideal_position = self._database.getPosi(pick_id)
        diff_x = current_pose.pose.position.x - ideal_position[0]
        diff_y = current_pose.pose.position.y - ideal_position[1]

        self._database.updateState(pick_id, "picked_x", current_pose.pose.position.x)
        self._database.updateState(pick_id, "picked_y", current_pose.pose.position.y)
        self._database.updateState(pick_id, "diff_x", diff_x)
        self._database.updateState(pick_id, "diff_y", diff_y)

    def save(self):
        top = "shot_point/" + self.FIELD
        self._database.save_csv(top + "_shoot.csv")
        diff_x = self._database.getColumn("diff_x")
        rospy.loginfo("diff_x: {}".format(np.average(diff_x)))
        diff_y = self._database.getColumn("diff_y")
        rospy.loginfo("diff_y: {}".format(np.average(diff_y)))


if __name__ == "__main__":
    rospy.init_node("picked_point", anonymous=True)
    node = PointLog()
    rospy.spin()
    node.save()
