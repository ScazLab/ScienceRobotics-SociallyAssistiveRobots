#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from clm_ros_wrapper.msg import VectorWithCertainty

jibo_pub = None

def user_face_callback(data):
    global jibo_pub
    x = str(data.position.x/1000.0)
    y = str(data.position.y/1000.0)
    z = str(data.position.z/1000.0)
    _to_robot = x + ',' + y + ',' + z

    rospy.loginfo(_to_robot)

    jibo_pub.publish(_to_robot)

def basic_publisher():
    global jibo_pub
    pub = rospy.Publisher('text_to_speech', String, queue_size=1)
    jibo_pub = rospy.Publisher('/sar/jibo/action', String, queue_size=1)
    rospy.init_node('basic_publisher', anonymous=True)
    rospy.Subscriber("clm_ros_wrapper/head_position_rf", VectorWithCertainty, user_face_callback)
    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
        #hello_str = "test"
        #rospy.loginfo(hello_str)
        #pub.publish(hello_str)
        #jibo_pub.publish('1,1,1')
        rate.sleep()

if __name__ == '__main__':
    try:
        basic_publisher()
    except rospy.ROSInterruptException:
        pass