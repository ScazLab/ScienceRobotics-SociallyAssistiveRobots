#!/usr/bin/env python

import threading
import rospy
from std_msgs.msg import String
from clm_ros_wrapper.msg import DetectedTarget
from geometry_msgs.msg import Vector3


jibo_pub = None
lock = threading.Lock()
user_head = None

def callback(data):
    global jibo_pub, lock, user_head

    _region = None
    _x = 1.0
    _y = 0.0
    _z = 1.0

    if data.region == DetectedTarget.ROBOT:
        _region = 'Robot'
        with lock:
            _x = user_head.x / 1000.0
            _y = user_head.y / 1000.0
            _z = (user_head.z + 200) / 1000.0
    elif data.region == DetectedTarget.SCREEN:
        _region = 'Screen'
        _x = 0
        _y = -0.3
        _z = 0.3
    elif data.region == DetectedTarget.OUTSIDE:
        _region = 'Outside'
    else:
        _region = 'NOT DEFINED'
        pass

    _to_robot = str(_x) + ',' + str(_y) + ',' + str(_z)
    jibo_pub.publish(_to_robot)

    rospy.loginfo('region: ' + _region + ' name: ' + data.name + ' distance: ' + str(data.distance))

def user_head_callback(data):
    global lock, user_head

    with lock:
        user_head = data

def seek_joint_attention():
    global jibo_pub

    jibo_pub = rospy.Publisher('/sar/jibo/lookat', String, queue_size=1)
    rospy.init_node('seek_joint_attention', anonymous=True)
    rospy.Subscriber("clm_ros_wrapper/detect_target", DetectedTarget, callback)
    rospy.Subscriber("clm_ros_wrapper/head_position_rf", Vector3, user_head_callback)

    rate = rospy.Rate(1) # 1hz

    while not rospy.is_shutdown():
        #jibo_pub.publish('1,1,1')
        rate.sleep()

if __name__ == '__main__':
    try:
        seek_joint_attention()
    except rospy.ROSInterruptException:
        pass