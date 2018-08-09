#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Vector3

jibo_pub = None
last_x = '1'
last_y = '0'
last_z = '1'
def callback(data):
    global jibo_pub, last_x, last_y, last_z

    x = str(data.x / 1000.0)
    y = str(data.y / 1000.0)
    z = str((data.z+100)  / 1000.0)

    if (data.x == 0.0) and (data.y==0.0) and (data.z==0.0):
        x = last_x
        y = last_y
        z = last_z
    else:
	last_x = x
	last_y = y
	last_z = z

    #cmhuang: testing
    #x = '1'
    #y = '0'
    #z = '1'

    _to_robot = x + ',' + y + ',' + z

    rospy.loginfo(_to_robot)

    jibo_pub.publish(_to_robot)

def face_following():
    global jibo_pub

    jibo_pub = rospy.Publisher('/sar/jibo/lookat', String, queue_size=1)
    rospy.init_node('face_following', anonymous=True)
    rospy.Subscriber("clm_ros_wrapper/head_position_rf", Vector3, callback)
    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
        #hello_str = "test"
        #rospy.loginfo(hello_str)
        #pub.publish(hello_str)
        #jibo_pub.publish('1,1,1')
        rate.sleep()

if __name__ == '__main__':
    try:
        face_following()
    except rospy.ROSInterruptException:
        pass
