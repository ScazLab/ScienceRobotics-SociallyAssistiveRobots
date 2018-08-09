#!/usr/bin/env python

import rospy
from std_msgs.msg import String

from sar_core.msg import CarTheta
from sar_core.msg import GameEvent
from sar_core.msg import CarCommand

class behavior_manager(object):

    def __init__(self):
        rospy.init_node('behavior_manager', anonymous=True)
        self._robot = rospy.get_param('~_p_robot')

        robot_info_topic = None
        robot_cmd_topic = None
        if self._robot == 'jibo':
            robot_info_topic = '/sar/jibo/info'
            robot_lookat_topic = '/sar/jibo/lookat'
            robot_speech_topic = '/sar/jibo/speech'
            robot_animation_topic = '/sar/jibo/animation'

        ## subscribers
        rospy.Subscriber("/sar/session_manager/status", String, self.session_manager_status_callback)

        rospy.Subscriber('/repairing_trust_car_theta', CarTheta, self.car_callback, queue_size=2)
        rospy.Subscriber('/car_example_command', CarCommand, self.car_command_callback, queue_size=2)
        rospy.Subscriber('/repairing_trust_game_event', GameEvent, self.game_event_callback, queue_size=2)

        rospy.Subscriber(robot_info_topic, String, self.jibo_callback)

        ## publishers
        self.robot_lookat_pub = rospy.Publisher(robot_lookat_topic, String, queue_size=1)
        self.robot_speech_pub = rospy.Publisher(robot_speech_topic, String, queue_size=1)
        self.robot_animation_pub = rospy.Publisher(robot_animation_topic, String, queue_size=1)

        # spin() simply keeps python from exiting until this node is stopped
        #rospy.spin()

    def run(self):
        #TODO: check perceptual JA event
        rate = rospy.Rate(1) # 1hz
        counter = 0
        while not rospy.is_shutdown():
            #hello_str = "test"
            #rospy.loginfo(hello_str)
            #pub.publish(hello_str)
            counter += 1
            if counter % 15 == 0:
                #self.robot_cmd_pub.publish('move')
                self.robot_lookat_pub.publish('1,1,1')
                #self.robot_animation_pub.publish('greeting.keys')
            elif counter % 15 == 5:
                self.robot_lookat_pub.publish('1,0,1') # center
            elif counter % 15 == 8:
                self.robot_animation_pub.publish('greeting.keys')
            elif counter %15 == 13:
                self.robot_speech_pub.publish('hello there!')
            rate.sleep()

    #### callbacks
    def session_manager_status_callback(self, data):
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        rospy.loginfo('session_manager_status = ' + data.data)

    def car_callback(self, data):
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        print 'message from car callback = ', data.theta

    def car_command_callback(self, data):
        print 'message from car command callback = ', data.command    

    def game_event_callback(self, data):
        print 'message from game event callback = ', data.game_event    

    def jibo_callback(self, data):
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        rospy.loginfo('jibo_callback gets = ' + data.data)

bm = behavior_manager()
bm.run()