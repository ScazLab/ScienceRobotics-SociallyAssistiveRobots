#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sar_game_command_msgs.msg import GameCommand, GameState
from sar_core.srv import GetGameLevel

import time
import random

class SimulatedSessionManager(object):
    def __init__(self):
        rospy.init_node('simulated_session_manager', anonymous=True)

        self.day_id = rospy.get_param('/sar/global/_g_day')
        self._f_game_is_done = False

        #rospy.Subscriber('/sar/game_state', GameState, self.game_state_callback, queue_size=1)

        self.game_command_publ = rospy.Publisher("/sar/game_command", GameCommand, queue_size=1)
        self.day_pub = rospy.Publisher("/sar/day_id", String, queue_size=10)

        rospy.Subscriber("/sar/game_state", GameState, self.game_state_callback)

        self.rate = rospy.Rate(0.1) #2 minutes: 0.0083

        self.TOTAL_NUM_DAYS = 10
        self.NUM_GAMES_PER_DAY = 2 # should be 3
        self.SET_OF_GAMES = {GameCommand.STORYTELLING, GameCommand.ROCKET_BARRIER} #add rest of games


    def run(self):
        while not rospy.is_shutdown():
            while self.day_id < self.TOTAL_NUM_DAYS:
                rospy.loginfo("========================== DAY [%d] =====================", self.day_id)
                games_today = random.sample(self.SET_OF_GAMES, self.NUM_GAMES_PER_DAY)
                for game in games_today:
                    game_command = GameCommand()
                    game_command.game = game#GameCommand.STORYTELLING
                    game_command.command = GameCommand.START
                    game_command.level = self.get_game_level_client(game_command.game)
                    self.game_command_publ.publish(game_command)
                    self.letswait()
                # print 'published game command to start game and now wait'

                rospy.Rate(0.1).sleep()
                self.day_id += 1
                # rospy.set_param('/sar/global/_g_day', self.day_id)
                self.day_pub.publish(str(self.day_id))
                # self.rate.sleep()
                rospy.Rate(0.1).sleep()


    def game_state_callback(self, data):
        _game_state = data.state
        if _game_state == GameState.END:
            self._f_game_is_done = True


    def letswait(self):
        while True:
            rospy.Rate(1).sleep()
            if self._f_game_is_done:
                break
        self._f_game_is_done = False

    def get_game_level_client(self, game_id):
        rospy.loginfo('game_id = '+ str(game_id))
        rospy.wait_for_service('/sar/get_game_level')
        try:
            get_game_level = rospy.ServiceProxy('/sar/get_game_level', GetGameLevel)
            level = get_game_level(game_id)
            rospy.loginfo('level = '+ str(level))
            return level.level
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


sm = SimulatedSessionManager()
sm.run()

