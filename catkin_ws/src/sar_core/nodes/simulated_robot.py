#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sar_robot_command_msgs.msg import RobotCommand, RobotState # ROS msgs for robot cmd
import json
import time

import os, subprocess, signal

from sar_core.msg import VSCommand, VSState

from server_communication import Packet
from server_communication import Communication

import random

class SimulatedRobot(object):
    def __init__(self):
        self.state = RobotState()
        #self.vs_command = VSCommand()
        self.state.is_playing_sound = True
        self.state.doing_action = True
        rospy.init_node('simulated_robot', anonymous=True)
        
        rospy.Subscriber("/sar/robot_command", RobotCommand, self.screen_callback)
        self.robot_state_publ = rospy.Publisher("/sar/robot_state", RobotState, queue_size=1)

        rospy.Subscriber("/sar/vsAction", String, self.vs_action_callback)
        self.vs_action_publ = rospy.Publisher("/sar/vsAction", String, queue_size=1)

        return
    def run(self):
        #INIT
        pack = Packet("Good Morning")
        pack.add([{"dataset" : "message", "numerical" : "Good Morning"}]).send()
        
        time.sleep(3)
        self.robot_state_publ.publish(self.state)
        self.state.is_playing_sound = False
        self.state.doing_action = False

        data = {}
        data['to'] = VSCommand.VS
        data['from'] = VSCommand.ROBOT
        data['action'] = VSCommand.ACTION_START
        data['day'] = "0"
        self.vs_action_publ.publish(json.dumps(data))


        _address='file:///home/waco001/catkin_ws/src/sar_core/resources/visual_schedule/index.html'
        #self.run_javascript_application(_address)
                
        while not rospy.is_shutdown():
            #MAIN LOOP
            current = time.time()
            if(current % .25 == 0):
                self.robot_state_publ.publish(self.state)
                
            if(current % 300 == 0):
                pack = Packet("Data Dump")
                pack.add([{"dataset" : "cpu_temp", "numerical" : str( 48 + (random.random()*7.5) )}]).send()
        #SHUTDOWN
        pack = Packet("Good Night")
        pack.add([{"dataset" : "message", "numerical" : "Good Night!"}]).send()
        return
    def vs_action_callback(self, data):
        data = json.loads(str(data.data))
        if(data['to'] == VSCommand.VS) and (data['action'] == VSCommand.ACTION_CLICK_MENU):
            print("CLICKED:")
            print(data['menu'])
        return
    def screen_callback(self, data):
        if(data.command == RobotCommand.DO):
            self.state.is_playing_sound = True
            self.state.doing_action = True
            print("RECIEVED SCREEN: \n" + data.properties)
            time.sleep(3) #sleep 3 seconds
            self.state.is_playing_sound = False
            self.state.doing_action = False
    
    def run_javascript_application(self, app_addr):
        _arg=' --app='
        _size=' --window-size=1920,1080'
        _position=' --window-position=0,0'
        _fullscreen = ' --kiosk'
        _cmd = 'google-chrome' + _fullscreen + _arg + app_addr # _position + _size
        return subprocess.Popen(_cmd, shell=True)


if __name__ == "__main__":
    robot = SimulatedRobot()
    robot.run()