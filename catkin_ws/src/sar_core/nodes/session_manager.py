#!/usr/bin/env python
import os, subprocess, signal
import random
import time

import rospy
from std_msgs.msg import String
from std_msgs.msg import Header # standard ROS msg header
from sar_core.msg import VSMessage, SystemState
from sar_robot_command_msgs.msg import RobotCommand, RobotState
from sar_game_command_msgs.msg import GameCommand, GameState
from sar_core.srv import GetGameLevel

import json
from pprint import pprint
from selenium import webdriver

from server_communication import Packet
from server_communication import Communication


class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


class session_manager(object):
    # constants
    STATE_INIT = 'STATE_INIT'
    STATE_DAILY_OPENING = 'STATE_DAILY_OPENING'
    STATE_GAME_SELECTION = 'STATE_GAME_SELECTION'
    STATE_PARENT_JOURNAL = 'STATE_PARENT_JOURNAL'
    STATE_EXIT = 'STATE_EXIT'
    STATE_WAIT = 'STATE_WAIT'

    #GAME_DURATION_LIMIT = 420 # 7 minutes
    # GAME_DURATION_LIMIT = 420 # 2 minutes
    DURATION_FOR_GAME_EXIT = 30
    NUM_GAME_TO_PLAY = 3

    game_id2str_dict = {GameCommand.STORYTELLING: 'storytelling', GameCommand.ROCKET_BARRIER: 'rocket_barrier',
                        GameCommand.GALACTIC_TRAVELER: 'galactic_traveler', GameCommand.SPACESHIP_TIDYUP: 'spaceship_tidyup',
                        GameCommand.ALIEN_CODES: 'alien_codes', GameCommand.HOUSE_PERSPECTIVE_TAKING: 'house_perspective_taking',
                        GameCommand.TRAIN_SEQUENCING: 'train_sequencing'}
    game_state2str_dict = {GameState.START: 'start', GameState.IN_PROGRESS: 'in_progress', GameState.PAUSED: 'paused',
                            GameState.USER_TIMEOUT: 'user_timeout', GameState.END: 'end', GameState.READY: 'ready', GameState.TUTORIAL: 'tutorial'}

    # TODO!!!: cmhuang: update this dict: only unity games
    game_id2app_keyword_dict = {GameCommand.STORYTELLING: 'StoryGame.x86_64', GameCommand.ROCKET_BARRIER: 'rocket_barrier_game.x86_64',
                        GameCommand.HOUSE_PERSPECTIVE_TAKING: 'HouseGame.x86_64',
                        GameCommand.TRAIN_SEQUENCING: 'TrainGame.x86_64'}

    # TODO: cmhuang: test differnt duration
    game_id2duration_limit_dict = {GameCommand.STORYTELLING: 240, GameCommand.ROCKET_BARRIER: 240,
                        GameCommand.GALACTIC_TRAVELER: 300, GameCommand.SPACESHIP_TIDYUP: 300,
                        GameCommand.ALIEN_CODES: 50, GameCommand.HOUSE_PERSPECTIVE_TAKING: 240,
                        GameCommand.TRAIN_SEQUENCING: 300}

    # note that due to json encoding and decoding, all keys and values are string
    cumulative_game_play_dict = {str(GameCommand.STORYTELLING): "0", str(GameCommand.ROCKET_BARRIER): "0",
                        str(GameCommand.GALACTIC_TRAVELER): "0", str(GameCommand.SPACESHIP_TIDYUP): "0",
                        str(GameCommand.ALIEN_CODES): "0", str(GameCommand.HOUSE_PERSPECTIVE_TAKING): "0",
                        str(GameCommand.TRAIN_SEQUENCING): "0"}

    game_name2filepath_dict = {}

    def __init__(self):
        rospy.init_node('session_manager', anonymous=True)

        self._p_sar_core_dir = rospy.get_param('/sar/global/_p_sar_core_dir')
        self._f_pilot_deployment = rospy.get_param('/sar/global/_pilot_deployment')
        self._reset_to_day1 = rospy.get_param('/sar/global/_reset_to_day1')
        self._ip_address = rospy.get_param('/sar/global/_ip_address')
        self._child_name = rospy.get_param('/sar/global/_child_name')
        self._send_signals_to_server = rospy.get_param('/sar/global/_send_signals_to_server')
        self._robot_id = rospy.get_param('/sar/global/_robot_id')
        self._default_notification_level = rospy.get_param('/sar/global/_default_notification_level')

        self.tts_pub = rospy.Publisher('text_to_speech', String, queue_size=1) # for external tts
        self.vs_pub = rospy.Publisher("/sar/vs", String, queue_size=10) #for actions between robot,js,vs
        self.robot_pub = rospy.Publisher("/sar/robot_command", RobotCommand, queue_size=2)
        self.game_pub = rospy.Publisher("/sar/game_command", GameCommand, queue_size=10)
        self.system_state_pub = rospy.Publisher("/sar/system/state", SystemState, queue_size=10)
        self.ss_signal_pub = rospy.Publisher("/sar/social_story/signal", String, queue_size=10)

        rospy.Subscriber("/sar/parent_journal", String, self.parent_journal_callback)
        rospy.Subscriber("/sar/vs", String, self.vs_callback)
        rospy.Subscriber("/sar/robot_state", RobotState, self.robot_state_callback)
        rospy.Subscriber("/sar/game_state", GameState, self.game_state_callback, queue_size=1)

        self.robot_scripts = None
        self.state = session_manager.STATE_INIT
        self.previous_state = None
        self._which_game = None
        self._game_duration_limit = 420 # default is 7 minutes
        self._first_time_play_do_tutorial = False
        self._p_game = None
        self._is_playing_game = False
        self._is_robot_ready = True
        self._t_game_begin = None
        self._n_game_played = 0
        self._f_has_asked_for_choice = False
        self._f_waiting_for_game_to_end = False
        self._f_waiting_for_pj_to_finish = False
        self._f_user_requested_start = False
        self._f_user_requested_start_once = True
        self._system_up_time = None
        self._session_end_time = None
        self._vs_closed_session = False
        self._game_is_ready_to_begin = False
        self._f_clean_start = True
        self._where_to_restart = 0
        self._todays_games = None
        self._previous_games = []
        self._previous_games_status = []

        self._script_testing = False # FISH ONLY for TESTING

        if self._f_pilot_deployment:
            # focus on three games for pilot deployment
            self.game_type_dict = {'emotional_understanding':[GameCommand.HOUSE_PERSPECTIVE_TAKING], # TODO: for the pilot test; STORYTELLING
                                'perspective_taking':[GameCommand.ROCKET_BARRIER],
                                'ordering_and_sequencing': [GameCommand.GALACTIC_TRAVELER]}
        else:
            # TODO: need to double check the following assignment
            self.game_type_dict = {'emotional_understanding':[GameCommand.STORYTELLING],
                                'perspective_taking':[GameCommand.ROCKET_BARRIER, GameCommand.HOUSE_PERSPECTIVE_TAKING],
                                'ordering_and_sequencing': [GameCommand.GALACTIC_TRAVELER, GameCommand.SPACESHIP_TIDYUP,
                                                            GameCommand.TRAIN_SEQUENCING]} #GameCommand.ALIEN_CODES,

        self.load_config_file()
        self.load_temp_log()


    def run(self):
        self.lets_wait(0.5)
        while not rospy.is_shutdown():

            if (self.state != session_manager.STATE_EXIT) and (self.state != session_manager.STATE_GAME_SELECTION) and (self.state != session_manager.STATE_WAIT): #for testing
                rospy.loginfo(bcolors.HEADER +'we are at the state of [' + self.state + ']' + bcolors.ENDC)

            if self.state == session_manager.STATE_INIT:
                self.send_system_state_msg(SystemState.SYSTEM_UP)
                self._system_up_time = time.strftime("%c")

                session_data = self.load_session_data()

                self.state = session_manager.STATE_DAILY_OPENING
                continue

            elif self.state == session_manager.STATE_DAILY_OPENING:
                rospy.loginfo("trying to run visual schedule")
                _p_js_app = self.run_javascript_application(self.visual_schedule_filepath, js_game=False)

                if self._f_clean_start:
                    # select games to play and pass the game ids to vs
                    self._todays_games = self.pick_todays_games()
                else:
                    self._todays_games = self._previous_games

                rospy.loginfo('today we are going to play [%s], [%s], [%s]',
                    self.game_id2str_dict[self._todays_games[0]], self.game_id2str_dict[self._todays_games[1]],
                    self.game_id2str_dict[self._todays_games[2]])
                # rospy.loginfo('today we are going to play [%d], [%d], [%d]',
                #     self._todays_games[0], self._todays_games[1], self._todays_games[2])

                # ensuring visual schedule js is ready
                self.lets_wait(3)

                JSdata = {}
                JSdata['to'] = VSMessage.VS
                #session manager is considered as robot when communicating with vs
                JSdata['from'] = VSMessage.ROBOT
                if self._f_clean_start:
                    JSdata['content'] = VSMessage.ACTION_START
                else:
                    JSdata['content'] = VSMessage.ACTION_RELOAD_SESSION_START
                    JSdata['game1_status'] = self._previous_games_status[0]
                    JSdata['game2_status'] = self._previous_games_status[1]
                    JSdata['game3_status'] = self._previous_games_status[2]
                JSdata['day'] = self._day
                JSdata['game1'] = self._todays_games[0]
                JSdata['game2'] = self._todays_games[1]
                JSdata['game3'] = self._todays_games[2]
                self.vs_pub.publish(json.dumps(JSdata))

                # set today's games to param server
                rospy.set_param('/sar/global/_today_game1', self._todays_games[0])
                rospy.set_param('/sar/global/_today_game2', self._todays_games[1])
                rospy.set_param('/sar/global/_today_game3', self._todays_games[2])

                self.save_temp_log(check_point=0)

                self.state = session_manager.STATE_WAIT
                continue

            elif self.state == session_manager.STATE_GAME_SELECTION:
                # finished all the game activity for today's session
                if self._n_game_played >= session_manager.NUM_GAME_TO_PLAY:
                    # closing today's session: load the collected piece of the day
                    self.lets_wait(3)
                    self.send_vs_msg(VSMessage.VS, VSMessage.ROBOT, VSMessage.ACTION_END)

                    # wait for the closing to finish
                    self.state = session_manager.STATE_WAIT
                    continue

                if self._which_game == None:
                    # cmhuang: TODO TODO
                    if (not self._f_has_asked_for_choice) and (self._n_game_played >= 1):
                        _robot_command = self.get_a_scripted_robot_command('transition_between_games')
                        self.send_multiple_robot_commands(_robot_command) #blocking
                        self._f_has_asked_for_choice = True
                        # enable menu
                        self.send_vs_msg(VSMessage.JS, VSMessage.ROBOT, VSMessage.ACTION_SHOW_MENU)
                    continue

                self._game_is_ready_to_begin = False
                # the user chose _which_game to play
                self.play_game(self._which_game)
                self._n_game_played += 1
                self._f_has_asked_for_choice = False

                if self._send_signals_to_server:
                    _msg = self.game_id2str_dict[self._which_game] + " is launched"
                    pack = Packet(_msg, self._default_notification_level, self._robot_id)
                    pack.add([{"dataset" : "message", "numerical" : "1"}]).send()

                self.state = session_manager.STATE_WAIT

            elif self.state == session_manager.STATE_PARENT_JOURNAL:
                p_parent_journal = self.run_javascript_application(self.parent_journal_filepath, js_game=False)
                self._f_waiting_for_pj_to_finish = True

                if self._send_signals_to_server:
                    _msg = "pj is launched"
                    pack = Packet(_msg, self._default_notification_level, self._robot_id)
                    pack.add([{"dataset" : "message", "numerical" : "1"}]).send()

                self.state = session_manager.STATE_WAIT
                continue

            elif self.state == session_manager.STATE_WAIT:
                self.lets_wait(seconds=0.1)

                # wait for the user to click the start button
                if self._f_user_requested_start == False:
                    continue
                else:
                    if self._f_user_requested_start_once:
                        self.send_vs_msg(VSMessage.JS, VSMessage.ROBOT, VSMessage.ACTION_LOAD_DEFAULT_BG)
                        self.state = session_manager.STATE_GAME_SELECTION
                        self._f_user_requested_start_once = False

                # after requesting to end the game, wait for the game to complete its ending process
                if self._f_waiting_for_game_to_end:
                    _now = rospy.get_time()
                    _duration_game_play = _now - self._t_game_begin
                    # force the game to exit after a minute
                    if _duration_game_play >= (self._game_duration_limit + session_manager.DURATION_FOR_GAME_EXIT):
                        rospy.loginfo('FORCE to kill game application [' + session_manager.game_id2str_dict[self._which_game] + ']')

                        if self._send_signals_to_server:
                            _msg = self.game_id2str_dict[self._which_game] + " is killed forcefully due to OVERTIME"
                            pack = Packet(_msg, self._default_notification_level, self._robot_id)
                            pack.add([{"dataset" : "message", "numerical" : "1"}]).send()

                        # force to kill the game
                        self.force_to_end_current_game()
                        # save to temp log
                        self.save_temp_log(check_point=self._n_game_played)
                        # reset game related parameters
                        self.reset_gaming_parameter()
                        self.state = session_manager.STATE_GAME_SELECTION
                    else:
                        continue

                if self._f_waiting_for_pj_to_finish:
                    continue

                if self._vs_closed_session:
                    _robot_command = self.get_a_scripted_robot_command('end_of_daily_session')
                    self.send_multiple_robot_commands(_robot_command)

                    # end of daily intervention session
                    self.send_system_state_msg(SystemState.SESSION_END)
                    self._session_end_time = time.strftime("%c")

                    self.lets_wait(3)
                    JSdata = {}
                    JSdata['to'] = VSMessage.JS
                    JSdata['from'] = VSMessage.ROBOT #session is considered as robot when communicating with vs
                    JSdata['content'] = VSMessage.ACTION_LOAD_IMAGE
                    JSdata['screen'] = "survey_time.png"
                    self.vs_pub.publish(json.dumps(JSdata))
                    _robot_command = self.get_a_scripted_robot_command('before_parent_journal')
                    self.send_multiple_robot_commands(_robot_command)
                    # robot goes to sleep mode
                    self.send_robot_command(RobotCommand.DO, "<jibo-sleep-mode>")
                    self.lets_wait(1.5)
                    # put robot to sleep
                    self.send_system_state_msg(SystemState.ROBOT_SLEEP)

                    self.state = session_manager.STATE_PARENT_JOURNAL

                if self._is_playing_game == True:
                    if self._p_game == None: # game finishes from within
                        _robot_command = self.get_a_scripted_robot_command('user_finished_game')
                        rospy.loginfo('%s is finished before its given time', self.game_id2str_dict[self._which_game])
                        self.send_multiple_robot_commands(_robot_command)
                        # save to temp log
                        self.save_temp_log(check_point=self._n_game_played)
                        self.reset_gaming_parameter()
                        self.state = session_manager.STATE_GAME_SELECTION
                    else:
                        if self._p_game.returncode == None: # the process is still alive
                            _now = rospy.get_time()
                            _duration_game_play = _now - self._t_game_begin
                            if _duration_game_play >= self._game_duration_limit:
                                # TODO: end robot behavior if any; how to do this gracefully?
                                _robot_command = self.get_a_scripted_robot_command('game_times_up')
                                self.send_multiple_robot_commands(_robot_command)
                                # TODO: this is not the best way to end a game.
                                self.lets_wait(3)
                                self.send_game_command(self._which_game, GameCommand.END)
                                self._f_waiting_for_game_to_end = True
                                rospy.loginfo('requesting game to end')
                                if self._send_signals_to_server:
                                    _msg = self.game_id2str_dict[self._which_game] + " is notified to end"
                                    pack = Packet(_msg, self._default_notification_level, self._robot_id)
                                    pack.add([{"dataset" : "message", "numerical" : "1"}]).send()
                        else:
                            # save to temp log
                            self.save_temp_log(check_point=self._n_game_played)
                            self.reset_gaming_parameter()
                            self.state = session_manager.STATE_GAME_SELECTION

                continue

            elif self.state == session_manager.STATE_EXIT:
                _session_start_time = rospy.get_param('/sar/global/session_start_time')
                today_session_data = {"day": self._day,
                                        "session_start_time": _session_start_time, "session_end_time": self._session_end_time,
                                        "system_up_time": self._system_up_time, "system_down_time": time.strftime("%c"),
                                        "cumulative_game_play": self.cumulative_game_play_dict}
                self.save_session_data(today_session_data)
                self.reset_temp_log()

                # very last: close the background vs-js application
                self.send_vs_msg(VSMessage.JS, VSMessage.ROBOT, VSMessage.ACTION_CLOSE)

                self.lets_wait(seconds=5)

                # signal all running nodes to shut off
                self.send_system_state_msg(SystemState.SYSTEM_DOWN)
                self.lets_wait(seconds=2)
                break

            self.lets_wait(seconds=0.01)

        # killall -- TODO: change later
        subprocess.Popen("killall roslaunch", shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)


    def load_config_file(self):
        with open(self._p_sar_core_dir+'/config/session_manager_config.json') as config_file:
            configurations = json.load(config_file)

        # games
        self.game_name2filepath_dict['storytelling'] = configurations['game_social_stories_filepath']
        self.game_name2filepath_dict['rocket_barrier'] = configurations['game_rocket_barrier_filepath']
        self.game_name2filepath_dict['house_perspective_taking'] = configurations['game_house_perspective_taking_filepath']
        self.game_name2filepath_dict['train_sequencing'] = configurations['game_train_sequencing_filepath']

        self.parent_journal_filepath = configurations['parent_journal_filepath']
        self.visual_schedule_filepath = configurations['visual_schedule_filepath']

        # load robot scripts
        with open(self._p_sar_core_dir+'/resources/scripts/session_manager_script.json') as robot_scripts_file:
            self.robot_scripts = json.load(robot_scripts_file)


    def load_temp_log(self):
        with open(self._p_sar_core_dir+'/logs/session_temp_log.json') as temp_log_file:
            temp_log = json.load(temp_log_file)

        _last_temp_log_date = temp_log['datetime']
        if _last_temp_log_date == "":
            self._f_clean_start = True
        else:
            self._f_clean_start = False
            rospy.loginfo('THIS IS A RELOAD SESSION: last play was at [%s]', _last_temp_log_date)

            self._previous_games.append(int(temp_log['game1']['id']))
            self._previous_games.append(int(temp_log['game2']['id']))
            self._previous_games.append(int(temp_log['game3']['id']))

            if temp_log['game1']['status'] == "":
                self._previous_games_status.append(0)
            else:
                self._previous_games_status.append(1)
            if temp_log['game2']['status'] == "":
                self._previous_games_status.append(0)
            else:
                self._previous_games_status.append(1)
            if temp_log['game3']['status'] == "":
                self._previous_games_status.append(0)
            else:
                self._previous_games_status.append(1)

            rospy.loginfo('THIS IS A RELOAD SESSION: last game setting [%s: %d], [%s: %d], [%s: %d]', \
                self.game_id2str_dict[self._previous_games[0]], self._previous_games_status[0], \
                self.game_id2str_dict[self._previous_games[1]], self._previous_games_status[1], \
                self.game_id2str_dict[self._previous_games[2]], self._previous_games_status[2])
            self._where_to_restart = sum(self._previous_games_status)
            self._n_game_played = self._where_to_restart


    def save_temp_log(self, check_point):
        temp_log = None
        with open(self._p_sar_core_dir+'/logs/session_temp_log.json') as temp_log_file:
            temp_log = json.load(temp_log_file)

        if check_point == 0: #after init
            temp_log['datetime'] = time.strftime("%c")
            temp_log['game1']['id'] = str(self._todays_games[0])
            temp_log['game2']['id'] = str(self._todays_games[1])
            temp_log['game3']['id'] = str(self._todays_games[2])
        elif check_point == 1 or check_point == 2 or check_point == 3: #finish a game
            if temp_log['game1']['id'] == str(self._which_game):
                temp_log['game1']['status'] = 'done'
            elif temp_log['game2']['id'] == str(self._which_game):
                temp_log['game2']['status'] = 'done'
            elif temp_log['game3']['id'] == str(self._which_game):
                temp_log['game3']['status'] = 'done'
        elif check_point == 4: #finish pj
            temp_log['pj'] = 'done'

        with open(self._p_sar_core_dir+'/logs/session_temp_log.json', 'w') as outfile:
            json.dump(temp_log, outfile)


    def reset_temp_log(self):
        temp_log = {"datetime": "", "game1": {"id":"", "status": ""}, "game2": {"id":"", "status": ""}, "game3": {"id":"", "status": ""}, "pj": ""}

        with open(self._p_sar_core_dir+'/logs/session_temp_log.json', 'w') as outfile:
            json.dump(temp_log, outfile)


    def save_session_data(self, today_data):
        session_data = {'sessions':[]}
        if self._reset_to_day1:
            session_data['sessions'].append(today_data)
        else:
            with open(self._p_sar_core_dir+'/logs/session_data.json') as session_data_file:
                session_data = json.load(session_data_file)
                session_data['sessions'].append(today_data)

        with open(self._p_sar_core_dir+'/logs/session_data.json', 'w') as outfile:
            json.dump(session_data, outfile)


    def load_session_data(self):
        if self._reset_to_day1:
            session_data = {'sessions':[]}
        else:
            with open(self._p_sar_core_dir+'/logs/session_data.json') as session_data_file:
                session_data = json.load(session_data_file)

        # accessing data
        day_data = session_data['sessions']
        if len(day_data) == 0:
            self._day = 1
        else:
            _yesterday = -1
            _yesterday_data = None
            for d in day_data:
                if d["day"] > _yesterday:
                    _yesterday = d["day"]
                    _yesterday_data = d
            self._day = _yesterday + 1
            # note: keys and values are all string
            self.cumulative_game_play_dict = _yesterday_data["cumulative_game_play"]
        # FISH ONLY for TESTING
        if self._script_testing:
            self._day = 1
            rospy.loginfo('today is day [%d]', self._day)
            rospy.set_param('/sar/global/_g_day', 1)
        else:
            # self._day = 0
            rospy.loginfo('today is day [%d]', self._day)
            rospy.set_param('/sar/global/_g_day', self._day)

        state = session_manager.STATE_INIT
        return session_data

    def pick_todays_games(self):
        # always pick the game with the lowest count to play
        game_to_play = []
        for key, value in self.game_type_dict.iteritems():
            _min_count = 30
            _min_count_game = None
            for game_id in value:
                game_id_str = str(game_id)
                cumulative_game_count = int(self.cumulative_game_play_dict[game_id_str])
                if cumulative_game_count < _min_count:
                    _min_count = cumulative_game_count
                    _min_count_game = int(game_id_str)
            game_to_play.append(_min_count_game)
        if GameCommand.STORYTELLING not in game_to_play:
            rospy.loginfo('NOT PLAYING SOCIAL STORY GAME TODAY')
            self.ss_signal_pub.publish('exit')
        else:
            rospy.loginfo('!!!!! PLAYING SOCIAL STORY GAME TODAY !!!!!')
        return game_to_play


    def get_a_scripted_robot_command(self, _situation):
        _scripts = self.robot_scripts[_situation]
        _choice = random.choice(_scripts)
        # rospy.loginfo(_choice)
        _cmd_list = []
        _cmd_list += _choice
        # rospy.loginfo(_cmd_list)
        return _cmd_list


    def get_game_level(self, _game):
        rospy.wait_for_service('/sar/get_game_level')
        try:
            _get_level = rospy.ServiceProxy('/sar/get_game_level', GetGameLevel)
            resp = _get_level(_game)
            rospy.loginfo("Playing game [%s] at level [%d]", self.game_id2str_dict[_game], resp.level)
            return resp.level
        except rospy.ServiceException, e:
            rospy.logwarn("Service call [/sar/get_game_level] failed: %s", e)


    def play_game(self, _game):
        self._p_game = None
        self._game_duration_limit = session_manager.game_id2duration_limit_dict[self._which_game]
        rospy.loginfo('now playing = [%s] for [%d] seconds', self.game_id2str_dict[_game], self._game_duration_limit)

        # get game level from personalization manger
        _game_level = self.get_game_level(_game)
        rospy.loginfo('_game_level = ' + str(_game_level))
        if _game == GameCommand.ROCKET_BARRIER or _game == GameCommand.HOUSE_PERSPECTIVE_TAKING or _game == GameCommand.TRAIN_SEQUENCING or _game == GameCommand.STORYTELLING:
            filepath = self.game_name2filepath_dict[self.game_id2str_dict[_game]]
            self._p_game = subprocess.Popen(filepath + ' -force-opengl', shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            rospy.loginfo(self.game_id2str_dict[_game] + ' game is launched.')
            self._is_playing_game = True
            # Get the current time in float seconds.
            self._t_game_begin = rospy.get_time()
            # set return code: A None value indicates that the process hasn't terminated yet.
            self._p_game.poll()
            # waiting for game to send the ready signal
            while self._game_is_ready_to_begin == False:
                self.lets_wait(0.1)
                continue
            self.lets_wait(0.5)
            # ensure the running application is in focus
            if _game == GameCommand.ROCKET_BARRIER:
                subprocess.Popen("wmctrl -a build", shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            elif _game == GameCommand.HOUSE_PERSPECTIVE_TAKING:
                subprocess.Popen("wmctrl -a HouseGame", shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            elif _game == GameCommand.TRAIN_SEQUENCING:
                subprocess.Popen("wmctrl -a TrainGame", shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            elif _game == GameCommand.STORYTELLING:
                subprocess.Popen("wmctrl -a StoryGame", shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

            if self._first_time_play_do_tutorial:
                self.send_game_command(self._which_game, GameCommand.PLAY_TUTORIAL, _game_level)
                self.send_game_command(self._which_game, GameCommand.PLAY_TUTORIAL, _game_level)
                self._first_time_play_do_tutorial = False
            else:
                self.send_game_command(self._which_game, GameCommand.START, _game_level)
                self.send_game_command(self._which_game, GameCommand.START, _game_level)
        elif _game == GameCommand.GALACTIC_TRAVELER or _game == GameCommand.ALIEN_CODES or _game == GameCommand.SPACESHIP_TIDYUP:
            self._p_game = self.run_javascript_application(self._ip_address+':8080', js_game=True)
            self._is_playing_game = True
            self._t_game_begin = rospy.get_time()
            self._p_game.poll()
            while self._game_is_ready_to_begin == False:
                self.lets_wait(0.1)
                continue

            subprocess.Popen("wmctrl -a Space", shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            # rospy.loginfo("now play")
            self.send_game_command(self._which_game, GameCommand.START, _game_level)
        else: # TODO
            rospy.logwarn('invalid game id')

        rospy.loginfo('now waitig for game to finish')
        #retval = _p_game.wait() # blocking: wait for the game to be finished


    def send_multiple_robot_commands(self, cmds):
       for cmd in cmds:
           # rospy.loginfo("threading cmd = " + cmd)
           self.send_robot_command(RobotCommand.DO, cmd)
           while self._is_robot_ready == False:
               self.lets_wait(0.1)
               continue


    def send_robot_command(self, _type, _cmd):
        # TODO: check if _is_robot_ready before sending?
        msg = RobotCommand()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.command = _type
        msg.properties = _cmd
        self.robot_pub.publish(msg)
        self._is_robot_ready = False


    def send_game_command(self, _game, _cmd, _level=1): #LEVELCHANGE
        msg = GameCommand()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.game = _game
        msg.command = _cmd
        if _cmd == GameCommand.START:
            msg.level = _level
        rospy.loginfo(msg)
        self.game_pub.publish(msg)


    def send_system_state_msg(self, sys_state):
        sys_state_msg = SystemState()
        sys_state_msg.header = Header()
        sys_state_msg.header.stamp = rospy.Time.now()
        sys_state_msg.system_state = sys_state
        self.system_state_pub.publish(sys_state_msg)


    def send_vs_msg(self, _to, _from, _content):
        data = {}
        data['to'] = _to
        # session manager is considered as robot when communicating with vs
        data['from'] = _from
        data['content'] = _content
        self.vs_pub.publish(json.dumps(data))


    def parent_journal_callback(self, data):
        pj_info = data.data
        # TODO: parse format
        if pj_info == 'done':
            rospy.loginfo('parent journal is done. moving to the exit state')
            self.save_temp_log(check_point=4)
            self.state = session_manager.STATE_EXIT
        else:
            pass
        rospy.loginfo(pj_info)


    def vs_callback(self, data):
        data = json.loads(data.data)
        #print(data)
        if(data['to'] == VSMessage.ROBOT):
            if(data['from'] == VSMessage.JS):
                if(data['content'] == VSMessage.STATE_GAME_SELECTED):
                    self._which_game = int(data['menu'])
                    # if _which_game is the first time to play, force tutorial
                    # if self._which_game == GameCommand.ROCKET_BARRIER or \
                    if self._which_game == GameCommand.TRAIN_SEQUENCING or \
                        self._which_game == GameCommand.HOUSE_PERSPECTIVE_TAKING:
                        if int(self.cumulative_game_play_dict[str(self._which_game)]) == 1:
                            self._first_time_play_do_tutorial = True

                    _robot_command = self.get_a_scripted_robot_command('loading_selected_game')
                    self.send_multiple_robot_commands(_robot_command)
                if(data['content'] == VSMessage.STATE_USER_START):
                    self._f_user_requested_start = True
            elif(data['from'] == VSMessage.VS):
                if(data['content'] == VSMessage.STATE_END):
                    self._vs_closed_session = True


    def robot_state_callback(self, data):
            self._is_robot_ready = not (data.is_playing_sound or data.doing_action)


    def game_state_callback(self, data):
        _game = data.game
        # if _game == self._which_game:
        _state = data.state
        rospy.loginfo('current game state = ' + self.game_state2str_dict[_state])
        if _state == GameState.READY:
            self._game_is_ready_to_begin = True
            rospy.loginfo("received signal: game is ready to begin")
        elif _state == GameState.END:
            _performance = data.performance
            # TODO: where do we log user perfomance??
            rospy.loginfo('user performance ' + _performance)
            # force to kill the running application
            self.force_to_end_current_game()

            if self._send_signals_to_server:
                _msg = self.game_id2str_dict[_game] + " is ended"
                pack = Packet(_msg, self._default_notification_level, self._robot_id)
                pack.add([{"dataset" : "message", "numerical" : "1"}]).send()

            # save to temp log
            self.save_temp_log(check_point=self._n_game_played)
            # reset game related parameters
            self.reset_gaming_parameter()
            self.state = session_manager.STATE_GAME_SELECTION
        elif _state == GameState.USER_TIMEOUT:
            self.send_game_command(_game, GameCommand.WAIT_FOR_RESPONSE)


    def reset_gaming_parameter(self):
        if self._which_game != None:
            self.cumulative_game_play_dict[str(self._which_game)] = str(int(self.cumulative_game_play_dict[str(self._which_game)]) + 1)
        self.lets_wait(seconds=0.1)

        self._is_playing_game = False
        self._t_game_begin = None
        self._which_game = None
        self._game_duration_limit = 420
        self._f_waiting_for_game_to_end = False


    def lets_wait(self, seconds):
        hz = 1 / float(seconds)
        rospy.Rate(hz).sleep()


    def force_to_end_current_game(self):
        if self._which_game == GameCommand.GALACTIC_TRAVELER or \
            self._which_game == GameCommand.SPACESHIP_TIDYUP or \
            self._which_game == GameCommand.ALIEN_CODES: #usc
            subprocess.Popen("wmctrl -c Space", shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        else: #unity games: yale and mit
            if self._which_game != None:
                self.force_to_kill_application(session_manager.game_id2app_keyword_dict[self._which_game])

        subprocess.Popen("wmctrl -a visual_schedule", shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)


    def force_to_kill_application(self, app_keyword):
        p = subprocess.Popen(['ps', '-A', 'e'], stdout=subprocess.PIPE)
        out, err = p.communicate()

        for line in out.splitlines():
            # print line
            if app_keyword in line:
                pid = int(line.split(None, 1)[0])
                # print pid
                os.kill(pid, signal.SIGKILL)

    def run_javascript_application(self, app_addr, js_game=False):
        _arg=' --app='
        _size=' --window-size=1920,1080 '
        _position=' --window-position=0,0 '
        _fullscreen = ' --kiosk'
        if js_game:
            # _cmd = 'google-chrome' + _position + _size + app_addr # _position + _size # _fullscreen
            # _cmd = 'firefox ' + app_addr
            _cmd = 'google-chrome' + _fullscreen + ' 192.168.2.3:8080'
            # _cmd = ['google-chrome', _fullscreen, '192.168.2.3:8080']
        else:
            # _cmd = 'google-chrome' + _position + _size + _arg + app_addr # _position + _size # _fullscreen
            _cmd = 'google-chrome' + _fullscreen + _arg + app_addr # _position + _size # _fullscreen
        # rospy.loginfo('_cmd = ' + _cmd)
        return subprocess.Popen(_cmd, shell=True)


if __name__ == "__main__":
    sm = session_manager()
    sm.run()
