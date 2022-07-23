#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import tf2_ros
import tf2_msgs
import geometry_msgs.msg


class ShootingBoxTransform:
    def __init__(self, shooting_box_center_raw):
        self._shooting_box_center_raw_np = np.array(shooting_box_center_raw)
        self._tfBuffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tfBuffer)
        self._br = tf2_ros.StaticTransformBroadcaster()
        # self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)

        name_space = "calibration/"
        self.BASE_FRAME = rospy.get_param(name_space + "base_frame")
        self.SHOOTING_BOX_FRAME = rospy.get_param(name_space + "shooting_box_frame")
        self.TARGET_BOX_FRAME = rospy.get_param(name_space + "target_box_frame")

    def get_calibrated_position(self, position_raw):
        ### position_rawとシューティングボックス中央からの相対座標を出す
        position_raw_np = np.array(position_raw)
        position_on_shooting_box_frame = (
            position_raw_np - self._shooting_box_center_raw_np
        )

        ### shooting_box フレームを用いて、一時的なtfを作成
        t = geometry_msgs.msg.TransformStamped()
        t.header.frame_id = self.SHOOTING_BOX_FRAME
        t.header.stamp = rospy.Time.now()
        t.child_frame_id = self.TARGET_BOX_FRAME
        t.transform.translation.x = position_raw_np[0]
        t.transform.translation.y = position_raw_np[1]
        t.transform.translation.z = position_raw_np[2]

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self._br.sendTransform(t)
        # tfm = tf2_msgs.msg.TFMessage([t])
        # self.pub_tf.publish(tfm)

        ### world座標にlookup transform
        tf1 = self.BASE_FRAME
        tf2 = self.TARGET_BOX_FRAME
        try:
            trans = self._tfBuffer.lookup_transform(
                tf1, tf2, rospy.Time(), rospy.Duration(10.0)
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            ### [TODO] error処理。現状どうしようもない
            return None

        ### world座標を[x,y,z]のlistにして送信
        ret = [
            trans.transform.translation.x,
            trans.transform.translation.y,
            trans.transform.translation.z,
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
