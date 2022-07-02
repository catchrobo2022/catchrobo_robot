#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander

from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped

class BiscoObstacle:
    def __init__(self):
        
        self._name = "bisco"

        self.BISCO_SIZE = 0.086, 0.029, 0.136
        self._LINK_NAME = "arm/link_tip"

        self._scene = moveit_commander.PlanningSceneInterface(synchronous=True)

        # self.cleanRviz()

    def cleanRviz(self):
        bisco_num = 27
        for i in range(bisco_num):
            self.release(i)
            
    def getName(self, id):
        return "{}{}".format(self._name, id)
    
    def attach(self, bisco_id):
        # [TODO] change for servo
        # touch_links = moveit_commander.RobotCommander().get_link_names("arm0")
        box_name = self.getName(bisco_id)
        self._scene.attach_box(self._LINK_NAME, box_name)

    def release(self, bisco_id):
        box_name = self.getName(bisco_id)
        self._scene.remove_attached_object(self._LINK_NAME, name=box_name)
        rospy.sleep(0.1)
        self._scene.remove_world_object(box_name)
        rospy.sleep(0.1)
    
    def addBox2Scene(self, database):

        rospy.wait_for_service("/get_planning_scene", timeout=10.0)
        rospy.wait_for_service("/apply_planning_scene", timeout=10.0)
        # p = PoseStamped()
        # p.header.frame_id = "world"
        # p.pose.position.z = _pub2gui0.8
        # p.pose.orientation.w = 1.0
        # size = 4, 4, 0.001
        # # rospy.sleep(0.1)
        # self._scene.add_box("cell", p, size)

        BISCO_SIZE = self.BISCO_SIZE
        bisco_num = database.getNum()
        # rospy.sleep(2)
        for i in range(bisco_num):
            if not database.isExist(i):
                continue
            p = PoseStamped()
            p.header.frame_id = "world"
            p.pose.position = database.getPosi(i)
            p.pose.position.z += BISCO_SIZE[2] / 2 + 0.0005
            p.pose.orientation.w = 1.0
            size = BISCO_SIZE[0], BISCO_SIZE[1], BISCO_SIZE[2] - 0.001
            # rospy.sleep(0.1)
            self._scene.add_box(self.getName(i), p, size)
        
    # def createBisco
        