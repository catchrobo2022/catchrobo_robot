#!/usr/bin/env python

_state = """Start"""

def set(msg):
  status = msg.data
  if status == 0:
    _state = """Start"""
  elif status == 1:
    _state = """Pause"""
  elif status == 2:
    _state = """Stop"""

  text = OverlayText()
  text.width = 80
  text.height = 30
  #text.height = 600
  text.left = 300
  text.top = 10
  text.text_size = 18
  text.line_width = 2
  text.font = "DejaVu Sans Mono"
  text.text = """%s""" % (_state)
  text.fg_color = ColorRGBA(25 / 255.0, 1.0, 240.0 / 255.0, 1.0)
  text.bg_color = ColorRGBA(0.0, 0.0, 0.0, 0.2)
  text_pub.publish(text)
  r.sleep()

try:
  from jsk_rviz_plugins.msg import *
except:
  import roslib;roslib.load_manifest("jsk_rviz_plugins")
  from jsk_rviz_plugins.msg import *

from std_msgs.msg import ColorRGBA, Float32, Int8
import rospy
import math
rospy.init_node("overlay_state")

text_pub = rospy.Publisher("text_sample", OverlayText)
value_pub = rospy.Publisher("value_sample", Float32)
state_pub = rospy.Subscriber("state", Int8, callback=set)
rate = 100
r = rospy.Rate(rate)

import random, math


while not rospy.is_shutdown():
  """
  text = OverlayText()
  text.width = 80
  text.height = 30
  #text.height = 600
  text.left = 10
  text.top = 10
  text.text_size = 18
  text.line_width = 2
  text.font = "DejaVu Sans Mono"
  """
  #text.text = """%s""" % (_state)
  """
  text.fg_color = ColorRGBA(25 / 255.0, 1.0, 240.0 / 255.0, 1.0)
  text.bg_color = ColorRGBA(0.0, 0.0, 0.0, 0.2)
  text_pub.publish(text)
  r.sleep()
  """

