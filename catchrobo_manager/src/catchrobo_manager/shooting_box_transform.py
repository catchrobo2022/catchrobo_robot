#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import tf2_ros
import tf2_msgs
import geometry_msgs.msg

from catchrobo_manager.calc_tf_diff import CalcTfDiff

class ShootingBoxTransform:
    def __init__(self, shooting_box_center_raw):
        self._shooting_box_center_raw_np = np.array(shooting_box_center_raw)
        self._tfBuffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tfBuffer)
        self._br = tf2_ros.StaticTransformBroadcaster()
        # self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)

        name_space = "calibration/"
        self.BASE_FRAME = rospy.get_param(name_space + "base_frame")
        self.SHOOTING_BOX_REAL_FRAME = rospy.get_param(name_space + "shooting_box_real_frame")
        self.SHOOTING_BOX_IDEAL_FRAME = rospy.get_param(
            name_space + "shooting_box_ideal_frame")

        self._calc_tf_diff=CalcTfDiff()

    def get_calibrated_position(self, position_raw):    
        ## position_rawとシューティングボックス中央からの相対座標を出す
        position_raw_np = np.array(position_raw)
        position_on_shooting_box_frame = (
            position_raw_np - self._shooting_box_center_raw_np
        )

        ### shooting_box フレームを用いて、一時的なtfを作成
        # t = geometry_msgs.msg.TransformStamped()
        # t.header.frame_id = self.SHOOTING_BOX_REAL_FRAME
        # t.header.stamp = rospy.Time.now()
        # t.child_frame_id = self.SHOOTING_BOX_IDEAL_FRAME
        # t.transform.translation.x = position_raw_np[0]
        # t.transform.translation.y = position_raw_np[1]
        # t.transform.translation.z = position_raw_np[2]

        # t.transform.rotation.x = 0.0
        # t.transform.rotation.y = 0.0
        # t.transform.rotation.z = 0.0
        # t.transform.rotation.w = 1.0

        # self._br.sendTransform(t)
        # tfm = tf2_msgs.msg.TFMessage([t])
        # self.pub_tf.publish(tfm)

        ### 実際のshooting_boxの位置からpositionまでのtransform
        ### world座標にlookup transform
        tf1 = self.SHOOTING_BOX_REAL_FRAME
        tf2 = self.SHOOTING_BOX_IDEAL_FRAME
        try:
            shooting_box_frame_diff = self._tfBuffer.lookup_transform(
                tf1, tf2, rospy.Time.now(), rospy.Duration(10.0)
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            ### [TODO] error処理。現状どうしようもない
            return None

        
        # shooting_box_frame = self._calc_tf_diff.transform_diff(tf2, tf1)
        

        # real shootingboxからposition
        shooting_box_frame = [
            shooting_box_frame_diff.translation.x, shooting_box_frame_diff.translation.y, shooting_box_frame_diff.translation.z]

        trans = position_on_shooting_box_frame-shooting_box_frame

        ### world座標を[x,y,z]のlistにして送信
        ret = [
            trans[0],
            trans[1],
            trans[2],
        ]
        return ret


if __name__ == "__main__":
    rospy.init_node("test_shooting_box_transform")
    rospy.sleep(1)
    node = ShootingBoxTransform([0, 0, 0])
    ret = node.get_calibrated_position([1, 0.5, 0])
    rospy.loginfo(ret)
    rospy.sleep(1)
    # rospy.spin()
