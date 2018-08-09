#!/usr/bin/env python
# verify if robot comands are properly processed and prepared for the jibo robot..

import rospy
from std_msgs.msg import String, Header
from sar_robot_command_msgs.msg import RobotCommand

class robot_command_verifier(object):

    def __init__(self):
        rospy.init_node('robot_command_verifier', anonymous=True)
        self.robot_pub = rospy.Publisher("/sar/robot_command", RobotCommand, queue_size=10)

    def run(self):
        while not rospy.is_shutdown():
            self.lets_wait(0.5)
            # TODO: read from a script file

            # single command test
            self.send_robot_command(RobotCommand.DO, "<lookat_child> Let's go [child-name], time to build your rocket. <lookat_guardian> [guardian-name], no looking!")
            self.lets_wait(2)
            self.send_robot_command(RobotCommand.DO, "<happy, nb> I am excited to play with you today! <lookat-screen, nb> Which game would you like to play first?")
            self.lets_wait(2)
            self.send_robot_command(RobotCommand.DO, "That was fun! <happy, nb> Let's play more! <lookat-screen, nb> What would you want to play next?")
            self.lets_wait(2)
            self.send_robot_command(RobotCommand.DO, "Thank you for your participation today! <happy, nb> See you tomorrow!")
            self.lets_wait(2)
            self.send_robot_command(RobotCommand.DO, "I enjoyed the game.")
            break

    def send_robot_command(self, _type, _cmd):
            msg = RobotCommand()
            msg.header = Header()
            msg.header.stamp = rospy.Time.now()
            msg.command = _type
            msg.properties = _cmd
            self.robot_pub.publish(msg)

    def lets_wait(self, seconds):
        hz = 1 / float(seconds)
        rospy.Rate(hz).sleep()

if __name__ == '__main__':
    try:
        rcv = robot_command_verifier()
        rcv.run()
    except rospy.ROSInterruptException:
        pass
