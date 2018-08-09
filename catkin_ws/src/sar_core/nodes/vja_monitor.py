#!/usr/bin/env python

import threading
import rospy
from std_msgs.msg import String
from sar_core.msg import SystemState, UserAttention, UserAttentions
import json
import os.path

class vja_monitor(object):
    _index_robot_attention_tracker_screen = 0
    _index_robot_attention_tracker_child = 1
    _index_robot_attention_tracker_parent = 2
    _index_vja_rc_tracker_screen = 0
    _index_vja_rc_tracker_robot = 1 # indication of mutual gaze between child and robot
    _index_vja_rc_tracker_parent = 2

    def __init__(self):
        rospy.init_node('vja_monitor', anonymous=True)
        self._p_sar_core_dir = rospy.get_param('/sar/global/_p_sar_core_dir')

        # rospy.Subscriber("/sar/perception/user_attention_target", String, self.user_attention_target_callback)
        # rospy.Subscriber("/sar/perception/parent_attention_target", String, self.parent_attention_target_callback)
        rospy.Subscriber('/sar/perception/attention_targets', UserAttentions, self.attention_targets_callback)
        rospy.Subscriber("/sar/jibo/robot_attention_target", String, self.robot_attention_target_callback)
        rospy.Subscriber("/sar/system/state", SystemState, self.system_state_callback)

        # self.lock_user_attention_target = threading.Lock()
        # self.lock_parent_attention_target = threading.Lock()
        self.lock_user_attention_targets = threading.Lock()
        self.lock_robot_attention_target = threading.Lock()

        self.user_attention_target = None
        self.parent_attention_target = None
        self.robot_attention_target = None
        self._f_system_down = False
        self._system_state = None

        # in percentage
        self.vja_rc_screen = 0.0
        self.vja_rc_parent = 0.0
        self.mg_rc = 0.0
        self.new_log = {}

        self.initialize_attention_trackers()


    def initialize_attention_trackers(self):
        self.robot_attention_tracker = []
        self.vja_rc_tracker = [] # visual joint attention between the robot and the child

        for i in range(SystemState.SYSTEM_UP, SystemState.SYSTEM_DOWN+1):
            self.robot_attention_tracker.append([0.0,0.0,0.0]) #screen, child, parent
            self.vja_rc_tracker.append([0.0,0.0,0.0]) #screen, robot, parent


    def system_state_callback(self, data):
        self._system_state = data.system_state
        if self._system_state == SystemState.SYSTEM_DOWN:
            self._f_system_down = True

    # def user_attention_target_callback(self, data):
    #     with self.lock_user_attention_target:
    #         self.user_attention_target = data.data


    # def parent_attention_target_callback(self, data):
    #     with self.lock_parent_attention_target:
    #         self.parent_attention_target = data.data

    def attention_targets_callback(self, data):
        with self.lock_user_attention_targets:
            for attention in data.attention:
                if attention.role == 'child':
                    self.user_attention_target = attention.region
                    # print "child " + self.user_attention_target
                elif attention.role == 'parent':
                    self.parent_attention_target = attention.region
                    # print "parent " + self.parent_attention_target


    def robot_attention_target_callback(self, data):
        with self.lock_robot_attention_target:
            self.robot_attention_target = data.data


    def save_vja_data(self):
        # TODO: calculate self.new_log
        self.new_log['robot_attention_tracker'] = self.robot_attention_tracker
        self.new_log['vja_rc_tracker'] = self.vja_rc_tracker

        _day = rospy.get_param('/sar/global/_g_day')
        self.new_log = {str(_day): self.new_log}

        out_log = None
        log_file = self._p_sar_core_dir+'/logs/vja_log.json'
        if os.path.isfile(log_file):
            with open(log_file) as vja_log_file:
                out_log = json.load(vja_log_file)
                out_log.update(self.new_log)
        else:
            out_log = self.new_log

        with open(self._p_sar_core_dir+'/logs/vja_log.json', 'w+') as vja_log_file:
            json.dump(out_log, vja_log_file)


    def run(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self._f_system_down:
                # TODO: test this
                self.save_vja_data()
                # rospy.loginfo('robot_attention_tracker = ' + str(self.robot_attention_tracker))
                # rospy.loginfo('vja_rc_tracker = ' + str(self.vja_rc_tracker))
                break

            if self._system_state == None:
                rate.sleep()
                continue

            # calculate and print out vja
            # if (self._system_state >= SystemState.SYSTEM_READY) and (self._system_state <= SystemState.ROBOT_SLEEP):
            if True:
                with self.lock_robot_attention_target and self.lock_user_attention_targets:
                    # TODO: during animation, the robot's attention should be a special case? IN-MOTION?
                    # color coding: http://stackoverflow.com/questions/287871/print-in-terminal-with-colors-using-python

                    # if (self.robot_attention_target == 'parent') and (self.parent_attention_target == 'robot'):
                    #     rospy.loginfo('\x1b[6;30;47m Having Mutual Gaze between [ROBOT and PARENT] \x1b[0m')
                    # if (self.robot_attention_target == 'child') and (self.user_attention_target == 'robot'):
                    #     rospy.loginfo('\x1b[6;37;45m Having Mutual Gaze between [ROBOT and CHILD] \x1b[0m')
                    # if (self.parent_attention_target == 'child') and (self.user_attention_target == 'parent'):
                    #     rospy.loginfo('\x1b[6;30;46m Having Mutual Gaze between [PARENT and CHILD] \x1b[0m')

                    # if (self.robot_attention_target == 'screen') and (self.user_attention_target == 'screen'):
                    #     rospy.loginfo('\x1b[6;30;42m Having Visual Joint Attention on [screen] between [ROBOT and CHILD] \x1b[0m')
                    # elif (self.robot_attention_target == 'parent') and (self.user_attention_target == 'parent'):
                    #     rospy.loginfo('\x1b[6;30;42m Having Visual Joint Attention on [parent] between [ROBOT and CHILD] \x1b[0m')

                    # if (self.parent_attention_target == 'screen') and (self.user_attention_target == 'screen'):
                    #     rospy.loginfo('\x1b[6;30;43m Having Visual Joint Attention on [screen] between [PARENT and CHILD] \x1b[0m')
                    # elif (self.parent_attention_target == 'robot') and (self.user_attention_target == 'robot'):
                    #     rospy.loginfo('\x1b[6;30;43m Having Visual Joint Attention on [robot] between [PARENT and CHILD] \x1b[0m')

                    # if self.user_attention_target == 'outside':
                    #     rospy.loginfo('\x1b[6;37;41m [CHILD] is not paying attention \x1b[0m')


                    # update the robot_attention_tracker
                    if self.robot_attention_target == 'screen':
                        self.robot_attention_tracker[self._system_state-1][self._index_robot_attention_tracker_screen] += 1
                    elif self.robot_attention_target == 'child':
                        self.robot_attention_tracker[self._system_state-1][self._index_robot_attention_tracker_child] += 1
                    elif self.robot_attention_target == 'parent':
                        self.robot_attention_tracker[self._system_state-1][self._index_robot_attention_tracker_parent] += 1

                    # update the vja_rc_tracker
                    if (self.robot_attention_target == 'screen') and (self.user_attention_target == 'screen'):
                        self.vja_rc_tracker[self._system_state-1][self._index_vja_rc_tracker_screen] += 1
                    elif (self.robot_attention_target == 'parent') and (self.user_attention_target == 'parent'):
                        self.vja_rc_tracker[self._system_state-1][self._index_vja_rc_tracker_parent] += 1
                    elif (self.robot_attention_target == 'child') and (self.user_attention_target == 'robot'):
                        # indication of mutual gaze between child and robot
                        self.vja_rc_tracker[self._system_state-1][self._index_vja_rc_tracker_robot] += 1

                    # calculate current vja_rc_screen, vja_rc_parent, and mg_rc (in percentage)
                    if self.robot_attention_tracker[self._system_state-1][self._index_vja_rc_tracker_screen] == 0:
                        self.vja_rc_screen = 0
                    else:
                        self.vja_rc_screen = self.vja_rc_tracker[self._system_state-1][self._index_vja_rc_tracker_screen] / \
                                             self.robot_attention_tracker[self._system_state-1][self._index_vja_rc_tracker_screen]

                    if self.robot_attention_tracker[self._system_state-1][self._index_vja_rc_tracker_parent] == 0:
                        self.vja_rc_parent = 0
                    else:
                        self.vja_rc_parent = self.vja_rc_tracker[self._system_state-1][self._index_vja_rc_tracker_parent] / \
                                             self.robot_attention_tracker[self._system_state-1][self._index_vja_rc_tracker_parent]

                    if self.robot_attention_tracker[self._system_state-1][self._index_vja_rc_tracker_robot] == 0:
                        self.mg_rc = 0
                    else:
                        self.mg_rc = self.vja_rc_tracker[self._system_state-1][self._index_vja_rc_tracker_robot] / \
                                             self.robot_attention_tracker[self._system_state-1][self._index_vja_rc_tracker_robot]

                    # if (self._system_state >= SystemState.SYSTEM_READY):
                        # rospy.loginfo('\033[94m vja_rc_screen = %f, vja_rc_parent = %f, mg_rc = %f \033[0m', self.vja_rc_screen, self.vja_rc_parent, self.mg_rc)
                        # rospy.loginfo('robot_attention_tracker = ' + str(self.robot_attention_tracker))
                        # rospy.loginfo('vja_rc_tracker = ' + str(self.vja_rc_tracker))

            rate.sleep()


if __name__ == '__main__':
    try:
        vm = vja_monitor()
        vm.run()
    except rospy.ROSInterruptException:
        pass
