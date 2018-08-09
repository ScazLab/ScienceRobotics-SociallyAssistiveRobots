#!/usr/bin/env python

import json
import threading, subprocess
import rospy
from sar_robot_command_msgs.msg import RobotCommand, RobotState # ROS msgs for robot cmd
from sar_core.msg import SystemState
from sar_core.msg import VSMessage
from std_msgs.msg import Header # standard ROS msg header
from std_msgs.msg import String

class dryrun_robot_scripts(object):

    def __init__(self):
        rospy.init_node('dryrun_robot_scripts', anonymous=True)

        self._n_day = 0
        self._is_robot_ready = False
        self._is_robot_connected = False
        self.lock_robot_ready = threading.Lock()

        self.robot_publ = rospy.Publisher("/sar/robot_command", RobotCommand, queue_size=10) #send screen to robot
        self.vs_pub = rospy.Publisher("/sar/vs", String, queue_size=10) #for actions between robot,js,vs

        rospy.Subscriber("/sar/system/state", SystemState, self.system_state_callback)
        rospy.Subscriber("/sar/robot_state", RobotState, self.robot_state_callback)

        with open('/home/sar/catkin_ws/src/sar_core/resources/scripts/session_manager_script.json') as transition_script_file:
            self.transition_scripts = json.load(transition_script_file)

        with open('/home/sar/catkin_ws/src/sar_core/resources/visual_schedule/usablestoryboard.json') as daily_script_file:
            self.daily_scripts = json.load(daily_script_file)


    def run(self):
        self.run_javascript_application('file:///home/sar/catkin_ws/src/sar_core/resources/visual_schedule/test.html', js_game=False)
        while True:
            rospy.sleep(1)
            if self._is_robot_connected:
                break

        self.dryrun_daily_scripts()
        # self.dryrun_transition_scripts()


    def dryrun_daily_scripts(self):
        while self._n_day < 31:
            rospy.loginfo('Day ' + str(self._n_day) + ' ==================================')
            intro_script = self.daily_scripts['day' + str(self._n_day)]['episode']
            # print intro_script

            rospy.loginfo('Daily introduction')
            _n_episode_item = 0
            while _n_episode_item < len(intro_script):
                _episode_item = intro_script[_n_episode_item]
                _command = _episode_item['command']
                _command = _command.encode('ascii', 'ignore')

                if "screen" in _episode_item:
                    _screen = _episode_item['screen']
                    self.send_screen_command(_screen.encode('ascii', 'ignore'))

                rospy.loginfo('command = [' + _command + ']')
                self.send_robot_command(RobotCommand.DO, _command)
                while True:
                    rospy.sleep(1)
                    with self.lock_robot_ready:
                        if self._is_robot_ready:
                            _n_episode_item += 1
                            break

            if self._n_day == 0:
                # no ending script
                pass
            else:
                # handling ending script
                ending_script = self.daily_scripts['day10' + str(self._n_day)]['episode']

                rospy.loginfo('Daily ending')
                _n_episode_item = 0
                while _n_episode_item < len(ending_script):
                    _episode_item = ending_script[_n_episode_item]
                    _command = _episode_item['command']
                    _command = _command.encode('ascii', 'ignore')

                    if "screen" in _episode_item:
                        _screen = _episode_item['screen']
                        self.send_screen_command(_screen.encode('ascii', 'ignore'))

                    rospy.loginfo('command = [' + _command + ']')
                    self.send_robot_command(RobotCommand.DO, _command)
                    while True:
                        rospy.sleep(1)
                        with self.lock_robot_ready:
                            if self._is_robot_ready:
                                _n_episode_item += 1
                                break

            self._n_day += 1


    def dryrun_transition_scripts(self):
        for key in self.transition_scripts:
            rospy.loginfo('key = [ ' + key + ' ]')
            possible_values = self.transition_scripts[key]
            for values in possible_values:
                for _command in values:
                    rospy.loginfo('command = [' + _command + ']')
                    self.send_robot_command(RobotCommand.DO, _command)
                    while True:
                        rospy.sleep(1)
                        with self.lock_robot_ready:
                            if self._is_robot_ready:
                                break


    def send_robot_command(self, _type, _cmd):
        msg = RobotCommand()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.command = _type
        msg.properties = _cmd
        self.robot_publ.publish(msg) #For robot
        with self.lock_robot_ready:
            self._is_robot_ready = False


    def send_screen_command(self, _screen):
        JSdata = {}
        JSdata['to'] = VSMessage.JS
        JSdata['from'] = VSMessage.ROBOT #session is considered as robot when communicating with vs
        JSdata['content'] = VSMessage.ACTION_LOAD_IMAGE
        JSdata['screen'] = _screen
        self.vs_pub.publish(json.dumps(JSdata))


    def robot_state_callback(self, data):
        with self.lock_robot_ready:
            self._is_robot_ready = not (data.is_playing_sound or data.doing_action)


    def system_state_callback(self, data):
        _sys_state = data.system_state
        if _sys_state == SystemState.SYSTEM_DOWN:
            self._f_system_down = True
        elif _sys_state == SystemState.ROBOT_WAKEUP:
            rospy.Rate(0.2).sleep() # five seconds
            self._is_robot_connected = True
            self._is_robot_ready = True


    def run_javascript_application(self, app_addr, js_game=False):
        _arg=' --app='
        _size=' --window-size=1920,1080 '
        _position=' --window-position=0,0 '
        _fullscreen = ' --kiosk'
        if js_game:
            # _cmd = 'google-chrome' + _position + _size + app_addr # _position + _size # _fullscreen
            # _cmd = 'firefox ' + app_addr
            _cmd = 'google-chrome' + _fullscreen + ' 192.168.1.3:8080' # _position + _size # _fullscreen
        else:
            # _cmd = 'google-chrome' + _position + _size + _arg + app_addr # _position + _size # _fullscreen
            _cmd = 'google-chrome' + _fullscreen + _arg + app_addr # _position + _size # _fullscreen
        # rospy.loginfo('_cmd = ' + _cmd)
        return subprocess.Popen(_cmd, shell=True)


if __name__ == "__main__":
    drs = dryrun_robot_scripts()
    drs.run()
