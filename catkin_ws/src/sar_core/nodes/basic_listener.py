#!/usr/bin/env python

import rospy
from std_msgs.msg import String

from sar_core.msg import CarTheta
from sar_core.msg import GameEvent
from sar_core.msg import CarCommand

from clm_ros_wrapper.msg import GazePointAndDirection

def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    print 'basic listener gets = ', data.data

def car_callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    print 'message from car callback = ', data.theta

def car_command_callback(data):
    print 'message from car command callback = ', data.command    

def game_event_callback(data):
    print 'message from game event callback = ', data.game_event    

def jibo_callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    print 'jibo_callback gets = ', data.data
    
def gaze_callback(data):
    rospy.loginfo(data)

def basic_listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('basic_listener', anonymous=True)

    rospy.Subscriber("t_session_manager_status", String, callback)
    #rospy.Subscriber("car_example_general", String, callback_car)
    rospy.Subscriber('/repairing_trust_car_theta', CarTheta, car_callback, queue_size=2)
    rospy.Subscriber('/car_example_command', CarCommand, car_command_callback, queue_size=2)
    rospy.Subscriber('/repairing_trust_game_event', GameEvent, game_event_callback, queue_size=2)

    rospy.Subscriber("/jibo_info", String, jibo_callback)
    rospy.Subscriber("clm_ros_wrapper/gaze_point_and_direction", GazePointAndDirection, gaze_callback)


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    basic_listener()
