#!/usr/bin/env python
# -*- coding: utf-8 -*-

# 下記のサイトをコピペした
# https: // qiita.com/ossyaritoori/items/80c8e0fa539ddff617e5

import rospy
import tf
import geometry_msgs



class CalcTfDiff:
    # Transform to homogeneous matrix
    def transform2homogeneousM(self,tfobj):
        # Quat to euler sxyz とあるが， XYZWの順番で良い。ちょっとわかりにくくないか？
        tfeul = tf.transformations.euler_from_quaternion(
            [tfobj.rotation.x, tfobj.rotation.y, tfobj.rotation.z, tfobj.rotation.w], axes='sxyz')
        # 並進量の記述
        tftrans = [tfobj.translation.x, tfobj.translation.y, tfobj.translation.z]
        tfobjM = tf.transformations.compose_matrix(angles=tfeul, translate=tftrans)
        # return
        return tfobjM

    def homogeneous2transform(self, Mat):
        scale, shear, angles, trans, persp = tf.transformations.decompose_matrix(
            Mat)
        quat = tf.transformations.quaternion_from_euler(
            angles[0], angles[1], angles[2])
        tfobj = geometry_msgs.msg.Transform()
        tfobj.rotation.x = quat[0]
        tfobj.rotation.y = quat[1]
        tfobj.rotation.z = quat[2]
        tfobj.rotation.w = quat[3]
        tfobj.translation.x = trans[0]
        tfobj.translation.y = trans[1]
        tfobj.translation.z = trans[2]
        return tfobj

    # Transform diff tf1 to 2
    def transform_diff(self, tf1, tf2):
        tf1M = self.transform2homogeneousM(tf1)
        tf2M = self.transform2homogeneousM(tf2)
        return self.homogeneous2transform(tf2M.dot(tf.transformations.inverse_matrix(tf1M)))


    # Pose version も作る
    def pose2homogeneousM(self, poseobj):
        try:
            # Quat to euler sxyz とあるが， XYZWの順番で良い。ちょっとわかりにくくないか？
            tfeul = tf.transformations.euler_from_quaternion(
                [poseobj.orientation.x, poseobj.orientation.y, poseobj.orientation.z, poseobj.orientation.w], axes='sxyz')
            # 並進量の記述
            tftrans = [poseobj.position.x, poseobj.position.y, poseobj.position.z]
            poseobjM = tf.transformations.compose_matrix(
                angles=tfeul, translate=tftrans)
            return poseobjM
        except:
            print("Input must be a pose object!")

    def homogeneous2pose(self, Mat):
        scale, shear, angles, trans, persp = tf.transformations.decompose_matrix(
            Mat)
        quat = tf.transformations.quaternion_from_euler(
            angles[0], angles[1], angles[2])
        poseobj = geometry_msgs.msg.Pose()
        poseobj.orientation.x = quat[0]
        poseobj.orientation.y = quat[1]
        poseobj.orientation.z = quat[2]
        poseobj.orientation.w = quat[3]
        poseobj.position.x = trans[0]
        poseobj.position.y = trans[1]
        poseobj.position.z = trans[2]
        return poseobj

    def pose_diff(self, p1, p2):
        p1M = self.pose2homogeneousM(p1)
        p2M = self.pose2homogeneousM(p2)
        return self.homogeneous2pose(p2M.dot(tf.transformations.inverse_matrix(p1M)))


# Geometry_msgs.msg.Transform()に関する関数
#   transform2homogeneousM：transformメッセージを同次変換行列（Homogeneous Matrix）に変換する
#   homogeneous2transform：同次変換行列をtransformメッセージに再変換
#   transform_diff(tf1, tf2)：2つのtransformのメッセージの差分をtransformメッセージで出力
# Geometry_msgs.msg.Pose()に関する関数
#   pose2homogeneousM：poseメッセージを同次変換行列（Homogeneous Matrix）に変換する
#   homogeneous2pose：同次変換行列をposeメッセージに再変換
#   pose_diff(p1, p2)：2つのposeのメッセージの差分をposeメッセージで出力
