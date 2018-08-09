#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Header # standard ROS msg header
from sar_core.msg import VSMessage, SystemState
from sar_robot_command_msgs.msg import RobotCommand, RobotState # ROS msgs for robot cmd
from sar_jibo_command_msgs.msg import JiboSpeech, JiboAnimation, JiboLookat
from sar_game_command_msgs.msg import GameCommand
import json
import threading
from pprint import pprint
import time

class VisualSchedule(object):
    def __init__(self):
        rospy.init_node('visual_schedule', anonymous=True)
        self.lock_robot_ready = threading.Lock()
        self.lock_vs_app_ready = threading.Lock()

        self._ORDER_REC = 0 #current index of _ORDER of what point we are in
        self._script_DATA = None #used to store the json in dict format

        self._run = False
        self._day = 0
        self._game1 = None
        self._game2 = None
        self._game3 = None
        self._is_robot_ready = False
        self._is_robot_connected = False
        self._is_vs_app_ready = False
        self._is_user_checking_in = True
        self._is_daily_opening = True
        self._is_daily_closing = False
        self._is_menu_selection = False
        self._f_menu_once = False
        self._f_robot_connected_once = False
        self._f_session_start = False
        self._f_system_down = False
        self._f_user_requested_start = False
        self._f_user_checked_in = False
        self._f_clean_start = True

        self._game_chosen = None

        self.__location__ = rospy.get_param('/sar/global/_p_sar_core_dir')
        with open(self.__location__ + '/resources/visual_schedule/usablestoryboard.json') as data_file:
            self._script_DATA = json.load(data_file)

        self.system_state_pub = rospy.Publisher("/sar/system/state", SystemState, queue_size=10)
        rospy.Subscriber("/sar/system/state", SystemState, self.system_state_callback)

        rospy.Subscriber("/sar/vs", String, self.vs_callback) #for actions between robot,js,vs
        self.vs_publ = rospy.Publisher("/sar/vs", String, queue_size=10) #for actions between robot,js,vs

        # rospy.Subscriber("/sar/jibo/verbose_state", String, self.jibo_verbose_state_callback)
        rospy.Subscriber("/sar/robot_state", RobotState, self.robot_state_callback) #/sar/robot_state
        self.robot_publ = rospy.Publisher("/sar/robot_command", RobotCommand, queue_size=10) #send screen to robot
        self.jibo_lookat_pub = rospy.Publisher('/sar/jibo/lookat', JiboLookat, queue_size=10) # TODO: move this to RobotCommand

        self.rate = rospy.Rate(10)


    def system_state_callback(self, data):
        _sys_state = data.system_state
        if _sys_state == SystemState.SYSTEM_DOWN:
            self._f_system_down = True
        elif _sys_state == SystemState.ROBOT_WAKEUP:
            self.lets_wait(seconds=2)
            self._is_robot_connected = True
            self._is_robot_ready = True


    def vs_callback(self, data):
        data = json.loads(data.data)
        #print(data)
        if(data['to'] == VSMessage.VS):
            if(data['from'] == VSMessage.JS):
                if(data['content'] == VSMessage.STATE_CONNECTED):
                    rospy.loginfo('vs-js is connected')
                    self._child_name = rospy.get_param('/sar/global/_child_name')
                    self._guardian1_name = rospy.get_param('/sar/global/_guardian1_name')
                    self._guardian2_name = rospy.get_param('/sar/global/_guardian2_name')
                    rospy.loginfo('_child_name = ' + self._child_name)
                    rospy.loginfo('_guardian1_name = ' + self._guardian1_name)
                    rospy.loginfo('_guardian2_name = ' + self._guardian2_name)
                    # set the default users
                    _data = {}
                    _data['to'] = VSMessage.JS
                    _data['from'] = VSMessage.VS
                    _data['content'] = VSMessage.ACTION_SET_DEFAULT_USERS
                    _data['child_name'] = self._child_name
                    _data['guardian1_name'] = self._guardian1_name
                    _data['guardian2_name'] = self._guardian2_name
                    self.vs_publ.publish(json.dumps(_data))
                    self.lets_wait(seconds=0.1)
                    self.vs_publ.publish(json.dumps(_data))
                elif(data['content'] == VSMessage.STATE_READY):
                    with self.lock_vs_app_ready:
                        self._is_vs_app_ready = True
                        rospy.loginfo('vs-js is ready')
                elif(data['content'] == VSMessage.STATE_GAME_SELECTED):
                    pass
                elif(data['content'] == VSMessage.STATE_USER_START):
                    self._f_user_requested_start = True
                elif(data['content'] == VSMessage.STATE_USER_CHECKED_IN):
                    self.send_robot_command(RobotCommand.DO, "Thank you.")
                    _today_guardian = data['guardian']
                    # set global variable: /sar/global/_guardian_name
                    rospy.set_param('/sar/global/_guardian_name', _today_guardian)
                    rospy.loginfo('todays guardian is '+_today_guardian)

                    self._f_user_checked_in = True

            elif(data['from'] == VSMessage.ROBOT): # messages from session manage
                if(data['content'] == VSMessage.ACTION_PAUSE):
                    self._run = False
                elif(data['content'] == VSMessage.ACTION_END):
                    # end session, start closing remarks
                    self.send_vs_message(VSMessage.JS, VSMessage.VS, VSMessage.ACTION_SHOW_MENU, _data = None)
                    self._ORDER_REC = 0
                    self._is_daily_closing = True
                    self._is_daily_opening = False
                    self._daily_date = self._script_DATA["day10"+str(self._day)]
                elif(data['content'] == VSMessage.ACTION_START):
                    self._run = True
                    self._day = data['day']
                    self._game1 = data['game1']
                    self._game2 = data['game2']
                    self._game3 = data['game3']
                    self._daily_date = self._script_DATA["day"+str(self._day)]
                    rospy.loginfo("vs is starting day = " + str(self._day))
                elif(data['content'] == VSMessage.ACTION_RELOAD_SESSION_START):
                    self._f_clean_start = False
                    self._run = True
                    self._day = data['day']
                    self._game1 = data['game1']
                    self._game2 = data['game2']
                    self._game3 = data['game3']
                    self._game1_status = data['game1_status']
                    self._game2_status = data['game2_status']
                    self._game3_status = data['game3_status']
                    self._is_daily_opening = False
                    self._is_daily_closing = False
                    self._is_menu_selection = True


    def robot_state_callback(self, data):
        '''Callback function when the TTS and facial expressions publish data to the topic. Uses the bools is_playing_sound and lock_recieved to add to the subscribers and recieved'''
        with self.lock_robot_ready:
            self._is_robot_ready = not (data.is_playing_sound or data.doing_action)


    def sendNextScreen(self):
        '''Function to use the _ORDER, _ORDER_REC, and _script_DATA to find out which screen is next and what data to publish'''
        data = self._daily_date
        if (self._ORDER_REC < len(data['episode'])):
            out = data['episode'][self._ORDER_REC]
            self.send_vs_message(VSMessage.JS, VSMessage.VS, VSMessage.ACTION_SEND_JS_DATA, _data = out)

            with self.lock_vs_app_ready:
                self._is_vs_app_ready = False

            self.send_robot_command(RobotCommand.DO, out['command'])

            self._ORDER_REC += 1
            return

        if self._is_daily_closing:
            self.send_vs_message(VSMessage.ROBOT, VSMessage.VS, VSMessage.STATE_END, _data = None)
            self._is_daily_closing = False
            return

        rospy.Rate(1).sleep()# test
        self._is_daily_opening = False
        if self._day > 0: # day 0 does not need to show the menu
            self._is_menu_selection = True


    def user_checking_in(self):
        if self._f_clean_start:
            out = '(pitch:9.0, pitchBandwidth:2.0, duration_stretch:1.07) Oh, <smile,nb> hello there! Please let me know who will be joining me today. <focuson-game,nb>'
        else:
            out = "(pitch:9.0, pitchBandwidth:2.0, duration_stretch:1.07) Let's pick up from where we left last time! Please let me know who will be joining me this time. <focuson-game,nb>"
        self.send_robot_command(RobotCommand.DO, out)
        self.wait_until_robot_is_ready()
        self.send_vs_message(VSMessage.JS, VSMessage.VS, VSMessage.ACTION_USER_CHECK_IN, _data = None)


    def send_vs_message(self, _to, _from, _content, _data):
        data = {}
        data['to'] = _to
        data['from'] = _from
        data['content'] = _content
        if _data != None:
            data['data'] = _data
        self.vs_publ.publish(json.dumps(data))


    def send_robot_command(self, _type, _cmd):
        msg = RobotCommand()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.command = _type
        msg.properties = _cmd
        self.robot_publ.publish(msg) #For robot
        with self.lock_robot_ready:
            self._is_robot_ready = False


    def send_jibo_lookat(self, _x, _y, _z, _duration=-1):
        msg = JiboLookat()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.x = _x
        msg.y = _y
        msg.z = _z
        msg.duration = _duration
        self.jibo_lookat_pub.publish(msg)


    def lets_wait(self, seconds):
        hz = 1 / float(seconds)
        rospy.Rate(hz).sleep()


    def wait_until_robot_is_ready(self):
        self.lets_wait(0.5) #500ms
        while True:
            with self.lock_robot_ready:
                if self._is_robot_ready:
                    break
            self.lets_wait(0.01) #10ms


    def run(self):
        '''function that has the main loop. first loops until all the subscribers return a "subscribe" publish (Now, only JS), and then moves to a loop that keeps continuing, but looks for the number of people who send "ready to go" for next screen'''
        while not rospy.is_shutdown():
            if self._f_system_down:
                break

            if self._is_robot_connected and (self._f_robot_connected_once == False):
                # the whole system is ready
                rospy.set_param('/sar/global/_p_whole_system_ready', "true")
                # load the default background image
                self.send_vs_message(VSMessage.JS, VSMessage.VS, VSMessage.ACTION_LOAD_WELCOME_BG, _data = None)
                self._f_robot_connected_once = True

            if self._run == False: pass
            elif self._f_user_requested_start == False: pass
            else:
                # TODO: ensure that users are there and ready
                if self._is_vs_app_ready and self._is_robot_ready:
                    # TODO: system is ready; should only do it once
                    # sys_state_msg = SystemState()
                    # sys_state_msg.header = Header()
                    # sys_state_msg.header.stamp = rospy.Time.now()
                    # sys_state_msg.system_state = SystemState.SYSTEM_READY
                    # self.system_state_pub.publish(sys_state_msg)

                    if self._is_user_checking_in:
                        self.user_checking_in()
                        self._is_user_checking_in = False
                        continue

                    if self._f_user_checked_in == False:
                        continue

                    if self._is_daily_opening:
                        if self._f_session_start == False:
                            _session_start_time = time.strftime("%c")
                            rospy.set_param('/sar/global/session_start_time', _session_start_time)

                            # beginning of daily intervention session
                            sys_state_msg = SystemState()
                            sys_state_msg.header = Header()
                            sys_state_msg.header.stamp = rospy.Time.now()
                            sys_state_msg.system_state = SystemState.SESSION_BEGIN
                            self.system_state_pub.publish(sys_state_msg)

                            self._f_session_start = True
                        self.sendNextScreen()
                    elif self._is_daily_closing:
                        self.sendNextScreen()
                    elif self._is_menu_selection and (self._f_menu_once == False):
                        if self._f_clean_start:
                            self.send_robot_command(RobotCommand.DO, "(pitch:9.0, pitchBandwidth:2.0, duration_stretch:1.07) <happy, nb> I'm really excited to play!")
                            self.wait_until_robot_is_ready()
                            self.send_robot_command(RobotCommand.DO, "(pitch:9.0, pitchBandwidth:2.0, duration_stretch:1.07) Which game would you like to play first today? <lookat-screen, b>")
                        else:
                            self.send_robot_command(RobotCommand.DO, "(pitch:9.0, pitchBandwidth:2.0, duration_stretch:1.07) <happy, nb> I'm really excited to play! Which game would you like to play now? <lookat-screen, b>")
                        self.lets_wait(seconds=1)
                        data = {}
                        data['to'] = VSMessage.JS
                        data['from'] = VSMessage.VS
                        if self._f_clean_start:
                            data['content'] = VSMessage.ACTION_OPEN_MENU
                            data['game1'] = self._game1
                            data['game2'] = self._game2
                            data['game3'] = self._game3
                        else:
                            _session_start_time = time.strftime("%c")
                            rospy.set_param('/sar/global/session_start_time', _session_start_time)

                            data['content'] = VSMessage.ACTION_RELOAD_OPEN_MENU
                            data['game1'] = self._game1
                            data['game2'] = self._game2
                            data['game3'] = self._game3
                            data['game1_status'] = self._game1_status
                            data['game2_status'] = self._game2_status
                            data['game3_status'] = self._game3_status

                        self.vs_publ.publish(json.dumps(data))
                        rospy.loginfo('Finished daily intro; now loading the menu.')
                        self._f_menu_once = True
            self.rate.sleep()


if __name__ == "__main__":
    vs = VisualSchedule()
    vs.run()
