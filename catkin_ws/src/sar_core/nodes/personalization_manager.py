#!/usr/bin/env python

import rospy
from std_msgs.msg import String

#from sar_core.msg import GamePerformanceEvent
from sar_game_command_msgs.msg import GameState
from sar_core.srv import GetGameLevel
from sar_core.msg import SystemState

import threading
import json

class personalization_manager(object):
    lock_current_levels = threading.Lock()

    received_performance = -1
    received_game_id = -1

    def __init__(self):
        rospy.init_node('personalization_manager', anonymous=True)

        self._p_sar_core_dir = rospy.get_param('/sar/global/_p_sar_core_dir')

        ## subscribers
        # need subscriber for the game performance events
        rospy.Subscriber('/sar/game_state', GameState, self.game_state_callback, queue_size=1)

        rospy.Subscriber("/sar/system/state", SystemState, self.system_state_callback)
        self._f_system_down = False

        # rospy.Subscriber("/sar/day_id", String, self.day_id_callback)

        ## publishers
        # need publisher to send new level of game (not sure what the topic should be called)
        # self.new_game_level_pub = rospy.Publisher('/game_level', String, queue_size=1)

        s = rospy.Service('/sar/get_game_level', GetGameLevel, self.handle_get_game_level)
        # print 'ready to provide new game level'

        self.log_file = self._p_sar_core_dir+'/config/performance_log.json'

        #variables for performance cutoffs
        self.LOW_THRESHOLD = 0.25
        self.MID_THRESHOLD = 0.75

        #lookup table for 3-level game: USC
        self.lookup_table3 = {'L1' : {'L':'0', 'M':'0', 'H':'1'},
                             'L2' : {'L':'-1', 'M':'0', 'H':'1'},
                             'L3' : {'L':'-1', 'M':'0', 'H':'0'}}

        #look-up table for 4-level game: Yale
        self.lookup_table4 = {'L1' : {'L':'0', 'M':'0', 'H':'1'},
                             'L2' : {'L':'-1', 'M':'0', 'H':'1'},
                             'L3' : {'L':'-1', 'M':'0', 'H':'1'},
                             'L4' : {'L':'-1', 'M':'0', 'H':'0'}}

        #lookup table for 10-level game: MIT
        self.lookup_table10 = {'L1' : {'L':'0', 'M':'0', 'H':'1'},
                             'L2' : {'L':'-1', 'M':'0', 'H':'1'},
                             'L3' : {'L':'-1', 'M':'0', 'H':'1'},
                             'L4' : {'L':'-1', 'M':'0', 'H':'1'},
                             'L5' : {'L':'-1', 'M':'0', 'H':'1'},
                             'L6' : {'L':'-1', 'M':'0', 'H':'1'},
                             'L7' : {'L':'-1', 'M':'0', 'H':'1'},
                             'L8' : {'L':'-1', 'M':'0', 'H':'0'}}


        self.game_measures_table = {'0' : {'child-accuracy' : -1.0, 'child-order-question-accuracy' : -1.0, 'child-emotion-question-accuracy' : -1.0},
                                    '1' : {'child-explainer-location-accuracy' : -1.0, 'child-explainer-piece-choice-accuracy' : -1.0, 'child-builder-location-accuracy' : -1.0, 'child-builder-piece-choice-accuracy' : -1.0},
                                    '2' : {'attempts_mean':-1.0, 'attempts_variance':-1.0, 'time_spent_mean':-1.0, 'time_spent_variance':-1.0, 'activity_with_most_attempts':-1.0, 'activity_child_spent_longest':-1.0, 'child_success_percentage':-1.0},
                                    '3' : {'attempts_mean':-1.0, 'attempts_variance':-1.0, 'time_spent_mean':-1.0, 'time_spent_variance':-1.0, 'activity_with_most_attempts':-1.0, 'activity_child_spent_longest':-1.0, 'child_success_percentage':-1.0},
                                    '4' : {'attempts_mean':-1.0, 'attempts_variance':-1.0, 'time_spent_mean':-1.0, 'time_spent_variance':-1.0, 'activity_with_most_attempts':-1.0, 'activity_child_spent_longest':-1.0, 'child_success_percentage':-1.0},
                                    '5' : {'child-guesser-percentage-correct':-1.0, 'child-builder-percentage-correct':-1.0},
                                    '6' : {'train-piece-errors':-1.0, 'train-order-errors':-1.0}}

        self.load_config_file()


    def run(self):
        rospy.Rate(0.2).sleep() # to ensure global parameters are available
        self.day_id = rospy.get_param('/sar/global/_g_day')
        self._reset_to_day1 = rospy.get_param('/sar/global/_reset_to_day1') # TODO:
        self.load_levels_data()
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            if self._f_system_down:
                break

            with self.lock_current_levels:
                if self.received_game_id != -1:
                    #update current_levels table with performance info and game_id
                    current_level = self.current_levels_table[str(self.received_game_id)]
                    new_level = self.get_new_level(self.received_game_id, current_level, self.received_performance)
                    rospy.loginfo('after updating, new level = %d', new_level)
                    self.current_levels_table[str(self.received_game_id)] = new_level
                    self.received_game_id = -1

            rate.sleep()


    def load_config_file(self):
        with open(self._p_sar_core_dir+'/config/measures_weights.json') as data_file:
            self.game_measures_weights = json.load(data_file)


    def load_levels_data(self):
        if self._reset_to_day1 or self.day_id == 1:
            self.current_levels_table = {
                "0": 1,
                "1": 1,
                "2": 1,
                "3": 1,
                "4": 1,
                "5": 1,
                "6": 1
            }
        else:
            with open(self.log_file) as data_file:
                daily_log = json.load(data_file)
                yesterday = self.day_id - 1
                self.current_levels_table = daily_log[str(yesterday)]["levels_table"]


    def log_daily_performance(self):
        if self._reset_to_day1 or self.day_id == 1:
            daily_log = {}

        else:
            with open(self.log_file) as data_file:
                daily_log = json.load(data_file)

        daily_log[self.day_id] = {"levels_table": self.current_levels_table, "measures": self.game_measures_table}
        with open(self.log_file, 'w+') as outfile:
            json.dump(daily_log, outfile, sort_keys = True, indent=4, ensure_ascii=False)

        rospy.loginfo('finished logging daily performance')


    def handle_get_game_level(self, request):
        new_game_level = self.current_levels_table[str(request.game_id)] #need to use lookup table based on request.game_id
        return {'level': new_game_level}


    #this function simply uses the lookup table to give back the new level
    #based on the current_level and the performance score
    def get_new_level(self, game_id, current_level, performance_string):
        #now performance is a json msg, so parse it first
        perf_msg = {}
        if len(performance_string) > 0:
            perf_msg = json.loads(performance_string)
        print perf_msg
        perf_sum = 0.0
        weights_sum = 0.0
        for key in perf_msg:
            if perf_msg[key] == None: #cmhuang: coping with USC null issue
                return current_level
            self.game_measures_table[str(game_id)][key] = perf_msg[key]
            weighted_val = perf_msg[key] * self.game_measures_weights[str(game_id)][key]
            perf_sum += weighted_val
            weights_sum += self.game_measures_weights[str(game_id)][key]

        if weights_sum == 0: #no performance info recorded, then keep level the same
            return current_level
        performance = (perf_sum / weights_sum)
        rospy.loginfo('averaging performance = %f', performance)

        #discretize performance value into H, M, L
        discrete_perf = ''
        if performance < self.LOW_THRESHOLD:
            discrete_perf = 'L'
        elif performance >= self.LOW_THRESHOLD and performance < self.MID_THRESHOLD:
            discrete_perf = 'M'
        else:
            discrete_perf = 'H'

        #use look-up table to know whether to go down, stay, or go up
        level_label = 'L' + str(current_level)
        result = ''
        try:
            if game_id == GameState.STORYTELLING:
                result = self.lookup_table10[level_label][discrete_perf]
            elif game_id == GameState.GALACTIC_TRAVELER or game_id == GameState.SPACESHIP_TIDYUP or game_id == GameState.ALIEN_CODES:
                result = self.lookup_table3[level_label][discrete_perf]
            else: # Yale games
                result = self.lookup_table4[level_label][discrete_perf]
        except KeyError:
            result = '0'

        #apply look-up table result to current level
        new_level = current_level + int(result)
        return new_level


    #### callbacks
    def game_state_callback(self, data):
        if data.state == GameState.END: #only parse game_state message in END state
            print 'personalization_manager callback gets game_id = ', data.game
            print 'personalization_manager callback gets performance = ', data.performance
            with self.lock_current_levels:
                self.received_game_id = data.game
                self.received_performance = data.performance
        else:
            return


    def system_state_callback(self, data):
        _sys_state = data.system_state
        if _sys_state == SystemState.SYSTEM_DOWN:
            self.log_daily_performance()
            self._f_system_down = True


    # def day_id_callback(self, data):
    #     self.log_daily_performance()

    #     self.day_id = int(data.data)
    #     with open(self.log_file) as data_file:
    #         daily_log = json.load(data_file)
    #     yesterday = self.day_id - 1
    #     self.current_levels_table = daily_log[str(yesterday)]["levels_table"]


if __name__ == "__main__":
    pm = personalization_manager()
    pm.run()
    #new_level = pm.get_new_level(0, 2, '{"child-explainer-location-accuracy" : 0.8,"child-explainer-piece-choice-accuracy" : 0.6}')
    #print 'new_level received is: ', new_level
