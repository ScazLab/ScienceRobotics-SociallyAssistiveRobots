#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sar_game_command_msgs.msg import GameCommand, GameState # ROS msgs for game cmd

from server_communication import Packet
from server_communication import Communication

import time
import random
import threading
import json

class SimulatedGame(object):

    lock_game_state = threading.Lock()
    game_state = -1 # which state
    game_level = 0

    game_state2str_dict = {GameState.START: 'start', GameState.IN_PROGRESS: 'in_progress', GameState.PAUSED: 'paused', GameState.USER_TIMEOUT: 'user_timeout', GameState.END: 'end'}


    def __init__(self):
        self.state = GameState()
        self.game_id = -1 #example game id is 0

        rospy.init_node('simulated_game', anonymous=True)

        rospy.Subscriber("/sar/game_command", GameCommand, self.game_command_callback)
        self.game_state_publ = rospy.Publisher("/sar/game_state", GameState, queue_size=1)

        #need publisher for server

        rospy.loginfo("hello")

        self.rate = rospy.Rate(1) #2 min

        return

    def run(self):
        while not rospy.is_shutdown():
            with self.lock_game_state:
                if self.game_state == GameState.START:
                    self.state.state = GameState.START
                    print 'game state is: ', self.game_state2str_dict[self.state.state]
                    # time.sleep(5) #sleep for a minute
                    performance = self.get_random_performance_scores()
                    #performance = {'child-emotion-question-accuracy' : random.random(), 'child-tom-question-accuracy' : random.random(), 'child-order-question-accuracy' : random.random()}
                    _performance = json.dumps(performance)
                    rospy.Rate(0.1).sleep()
                    self.state.state = GameState.END #end game
                    print 'after performance, game state is: ', self.game_state2str_dict[self.state.state]
                    self.state.performance = _performance
                    self.state.game = self.game_id

                    self.game_state = GameState.END
                    self.game_state_publ.publish(self.state)
                    # pack = Packet("Game Test", 0)
                    # pack.add([{"dataset" : "sar_game_" + str(self.game_id) + "_difficulty", "numerical" : self.game_level},{"dataset" : "sar_game_" + str(self.game_id) + "_performance", "numerical" : performance * 100}]).send()
            self.rate.sleep()


    def get_random_performance_scores(self):
        performance = {}
        if self.game_id == GameCommand.STORYTELLING:
            performance = {'child-emotion-question-accuracy' : random.random(), 'child-tom-question-accuracy' : random.random(), 'child-order-question-accuracy' : random.random()}
        elif self.game_id == GameCommand.ROCKET_BARRIER:
            performance = {'child-explainer-location-accuracy' : random.random(), 'child-explainer-piece-choice-accuracy' : random.random()}
        elif self.game_id == GameCommand.GALATIC_BARRIER:
            performance = {}
        elif self.game_id == GameCommand.SPACESHIP_TIDYUP:
            performance = {}
        elif self.game_id == GameCommand.ALIEN_CODES:
            performance = {}
        elif self.game_id == GameCommand.HOUSE_PERSPECTIVE_TAKING:
            performance = {}
        elif self.game_id == GameCommand.TRAIN_SEQUENCING:
            performance = {}

        return performance


    def game_command_callback(self, data):
        # rospy.loginfo('game id = '+ str(data.game))
        #if data.game == self.game_id: #for simulated test, parse start message with any game_id
        self.game_id = data.game
        if data.command == GameCommand.START: #start command
            with self.lock_game_state:
                self.game_state = GameState.START
                self.game_level = data.level


if __name__ == "__main__":
    game = SimulatedGame()
    game.run()



