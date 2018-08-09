#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sar_robot_command_msgs.msg import RobotCommand, RobotState # ROS msgs for robot cmd
from sar_jibo_command_msgs.msg import JiboSpeech, JiboAnimation, JiboLookat
import json
import time

import os, subprocess, signal

class SimulatedJIBO(object):
    def __init__(self):
        self.state = RobotState()
        #self.vs_command = VSCommand()
        self.state.is_playing_sound = False
        self.state.doing_action = False
        rospy.init_node('simulated_jibo', anonymous=True)

        rospy.Subscriber("/sar/jibo/lookat", JiboLookat, self.jibo_lookat_cmd_callback)
        rospy.Subscriber("/sar/jibo/speech", JiboSpeech, self.jibo_speech_cmd_callback)
        rospy.Subscriber("/sar/jibo/animation", JiboAnimation, self.jibo_animation_cmd_callback)

        self.tts_pub = rospy.Publisher('text_to_speech', String, queue_size=1) # for external tts
        self.jibo_state_pub = rospy.Publisher("/sar/jibo/state", RobotState, queue_size=1)


    def run(self):
        while not rospy.is_shutdown():
            current = time.time()
            if(current % .25 == 0): # HOW frequnt??
                self.jibo_state_pub.publish(self.state)


    def jibo_lookat_cmd_callback(self, data):
        self.state.doing_action = True
        rospy.loginfo("receiving jibo lookat command: [%f,%f,%f]", data.x, data.y, data.z)
        self.lets_wait(seconds=2)
        self.state.doing_action = False


    def jibo_speech_cmd_callback(self, data):
        self.state.is_playing_sound = True
        rospy.loginfo("receiving jibo speech command: [%s]", data.speech_content)
        self.tts_pub.publish(data.speech_content)
        self.lets_wait(seconds=5)
        self.state.is_playing_sound = False


    def jibo_animation_cmd_callback(self, data):
        self.state.doing_action = True
        rospy.loginfo("receiving jibo animation command: [%s]", data.animation_name)
        self.lets_wait(seconds=2)
        self.state.doing_action = False


    def lets_wait(self, seconds):
        hz = 1 / float(seconds)
        rospy.Rate(hz).sleep()


if __name__ == "__main__":
    jibo = SimulatedJIBO()
    jibo.run()
