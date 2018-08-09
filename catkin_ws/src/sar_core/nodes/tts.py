#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

voice = 'voice_kal_diphone'
soundhandle = SoundClient()

def callback(data):
	global soundhandle, voice
	soundhandle.say(data.data, voice)
	rospy.sleep(1)

def tts():
	rospy.init_node('tts', anonymous=True)
	rospy.Subscriber("text_to_speech", String, callback)
	rospy.spin()

if __name__ == '__main__':
    tts()