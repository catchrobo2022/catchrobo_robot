#!/usr/bin/env python3

# in Terminal
# sudo apt install ros-melodic-sound-play
# roslaunch sound_play soundplay_node.launch

# if you want to use wav file, put your file /opt/ros/melodic/share/sound_play/sounds/


import rospy, os, sys
import rospkg
from std_msgs.msg import Int8
from catchrobo_msgs.msg import ErrorCode

# from std_msgs.msg import Time

from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

import time


class Listener:
    def __init__(self):
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path("catchrobo_sound")
        self._sound_dir = pkg_path + "/sound/"

        self.MANUAL_ON_BUTTON = self._sound_dir + "potate.wav"
        self.AUTO_MANUAL_ON = self._sound_dir + "potate.wav"
        self.ARRIVE = self._sound_dir + "potate.wav"

        rospy.Subscriber("manual_command", Int8, self.ManualCommandCallback)
        rospy.Subscriber("auto2manual_command", Int8, self.Auto2ManualCallback)
        rospy.Subscriber("error", ErrorCode, self.ErrorCallback)

        self.soundhandle = SoundClient()
        rospy.sleep(1)
        self.soundhandle.stopAll()
        self.start = time.time()

        pass

    def ManualCommandCallback(self, msg):
        if msg.data == 1:
            self.soundhandle.playWave(self.MANUAL_ON_BUTTON)

    def Auto2ManualCallback(self, msg):
        if msg.data == 1:
            self.soundhandle.playWave(self.AUTO_MANUAL_ON)

    def ErrorCallback(self, msg):
        if msg.data == 1:
            # self.soundhandle.play(SoundRequest.NEEDS_PLUGGING)
            self.soundhandle.playWave(self.ARRIVE)  # change your file

    def main(self):
        rospy.loginfo(
            "This script will run continuously until you hit CTRL+C, testing various sound_node sound types."
        )
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node("catchrobo_sound", anonymous=True)
    listener = Listener()
    listener.main()
