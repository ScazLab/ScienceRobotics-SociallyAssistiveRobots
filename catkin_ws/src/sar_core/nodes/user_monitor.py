#!/usr/bin/env python
import threading

import rospy

from std_msgs.msg import String
from std_msgs.msg import Header # standard ROS msg header
from geometry_msgs.msg import Vector3
from clm_ros_wrapper.msg import VectorWithCertainty, GazePointAndDirection
from sar_robot_command_msgs.msg import RobotCommand
from clm_ros_wrapper.msg import DetectedTarget, DetectedTargets
from sar_core.srv import GetUserHeadPosition
from sar_jibo_command_msgs.msg import JiboSpeech, JiboAnimation, JiboLookat
from sar_core.msg import SystemState, UserAttentions, UserAttention

class user_monitor(object):
    DISENGAGED_TIMEOUT = 10 #10 seconds
    TARGET_DISTANCE = 50 #50 mm
    TRACKING_TIMEOUT = 1000 #1000 ms
    CLM_CERTAINTY_THRESHOLD = 0.65

    current_focus = None
    user_state = None

    # lock_detected_target = threading.Lock()
    # lock_parent_detected_target = threading.Lock()
    lock_detected_targets = threading.Lock()
    lock_user_head_position_rf = threading.Lock()
    lock_user_head_pose_wf = threading.Lock()

    last_detected_region = None
    last_parent_detected_region = None
    last_user_head_x = 1.0 #m
    last_user_head_y = -0.4 #m
    last_user_head_z = 0.8 #m

    last_user_head_position_wf = Vector3()# should be Point
    last_user_head_orientation_wf = Vector3()


    def __init__(self):
        rospy.init_node('user_monitor', anonymous=True)

        self.TEST_FLAG_FACE_FOLLOWING = rospy.get_param('~test_flag_face_following')
        self.TEST_FLAG_JA_SEEKING = rospy.get_param('~test_flag_ja_seeking')

        self.robot_command_pub = rospy.Publisher('/sar/robot_command', RobotCommand, queue_size=10)

        rospy.Service('/sar/perception/get_user_head_position', GetUserHeadPosition, self.get_user_head_position)
        # TODO: which node subscribe to the following topic?
        self.user_head_rf_pub = rospy.Publisher('/sar/perception/user_head_position_rf', Vector3, queue_size=10)

        # self.user_attention_target_pub = rospy.Publisher('/sar/perception/user_attention_target', String, queue_size=10)
        # self.parent_attention_target_pub = rospy.Publisher('/sar/perception/parent_attention_target', String, queue_size=10)

        self.attention_targets_pub = rospy.Publisher('/sar/perception/attention_targets', UserAttentions, queue_size=10)

        # rospy.Subscriber("/sar/perception/detect_target", DetectedTarget, self.detected_target_callback)
        # rospy.Subscriber("/sar/perception/parent_detect_target", DetectedTarget, self.parent_detected_target_callback)

        rospy.Subscriber("/sar/perception/detect_targets", DetectedTargets, self.detected_targets_callback)
        rospy.Subscriber("/sar/perception/head_position_rf", VectorWithCertainty, self.user_head_position_rf_callback)

        rospy.Subscriber("/sar/system/state", SystemState, self.system_state_callback)
        self._f_system_down = False

        self.rate = rospy.Rate(2) # 10hz 100ms


    def run(self):
        while not rospy.is_shutdown():
            if self._f_system_down:
                break

            if self.TEST_FLAG_FACE_FOLLOWING:
                self.test_jibo_lookat_user()
            elif self.TEST_FLAG_JA_SEEKING:
                with self.lock_detected_targets:
                    if self.last_detected_region == None:
                        continue
                    rospy.loginfo("current user attention target: "+ self.last_detected_region)

            self.rate.sleep()


    def test_jibo_lookat_user(self):
        self.send_robot_command(RobotCommand.DO, "<lookat_child>")


    # for the robot to request for the latest user head position
    def get_user_head_position(self, request):
        with self.lock_user_head_position_rf:
            return {'x': self.last_user_head_x, 'y': self.last_user_head_y, 'z': self.last_user_head_z}


    def user_head_position_rf_callback(self, data):
        # instead of using a faked position for kid's head, use the previous detected position...
        if (data.position.x == 0.0) and (data.position.y==0.0):# and (data.position.z==0.0):
            _head_pos = Vector3()
            _head_pos.x = self.last_user_head_x#1
            _head_pos.y = self.last_user_head_y#-0.4
            _head_pos.z = self.last_user_head_z#0.8
            self.user_head_rf_pub.publish(_head_pos)
            return
        if (data.certainty < self.CLM_CERTAINTY_THRESHOLD):
            _head_pos = Vector3()
            _head_pos.x = self.last_user_head_x#1
            _head_pos.y = self.last_user_head_y#-0.4
            _head_pos.z = self.last_user_head_z#0.8
            self.user_head_rf_pub.publish(_head_pos)
            return
        with self.lock_user_head_position_rf:
            self.last_user_head_x = data.position.x #in meters
            self.last_user_head_y = data.position.y #in meters
            self.last_user_head_z = data.position.z #in meters
            _head_pos = Vector3()
            _head_pos.x = self.last_user_head_x
            _head_pos.y = self.last_user_head_y
            _head_pos.z = self.last_user_head_z
            self.user_head_rf_pub.publish(_head_pos)

    def detected_targets_callback(self, data):
        with self.lock_detected_targets:
            attentions = UserAttentions()
            child_attention = UserAttention()
            child_attention.role = 'child'
            parent_attention = UserAttention()
            parent_attention.role = 'parent'
            for user in data.targets:
                if user.role == DetectedTarget.CHILD_ROLE:
                    self.last_detected_region = None
                    if user.region == DetectedTarget.ROBOT:
                        child_attention.region = 'robot'
                    elif user.region == DetectedTarget.PARENT:
                        child_attention.region = 'parent'
                    elif user.region == DetectedTarget.SCREEN:
                        child_attention.region = 'screen'
                    elif user.region == DetectedTarget.OUTSIDE:
                        child_attention.region = 'outside'
                    elif user.region == DetectedTarget.NONE:
                        child_attention.region = 'no detection'
                    attentions.attention.append(child_attention)
                    self.last_detected_region = child_attention.region
                elif user.role == DetectedTarget.PARENT_ROLE:
                    if user.region == DetectedTarget.ROBOT:
                        parent_attention.region = 'robot'
                    elif user.region == DetectedTarget.CHILD:
                        parent_attention.region = 'child'
                    elif user.region == DetectedTarget.SCREEN:
                        parent_attention.region = 'screen'
                    elif user.region == DetectedTarget.OUTSIDE:
                        parent_attention.region = 'outside'
                    elif user.region == DetectedTarget.NONE:
                        parent_attention.region = 'no detection'
                    attentions.attention.append(parent_attention)
            self.attention_targets_pub.publish(attentions)


    # def detected_target_callback(self, data): # ideally 30hz about 33 mm
    #     # TODO: utilizing data.name and data.distance
    #     with self.lock_detected_target:
    #         if data.region == DetectedTarget.ROBOT:
    #             self.last_detected_region = 'robot'
    #         elif data.region == DetectedTarget.PARENT:
    #             self.last_detected_region = 'parent'
    #         elif data.region == DetectedTarget.SCREEN:
    #             self.last_detected_region = 'screen'
    #         elif data.region == DetectedTarget.OUTSIDE:
    #             self.last_detected_region = 'outside'
    #         elif data.region == DetectedTarget.NONE:
    #             self.last_detected_region = 'no detection'
    #         # rospy.loginfo("current user attention target: "+ self.last_detected_region)
    #     self.user_attention_target_pub.publish(self.last_detected_region)


    # def parent_detected_target_callback(self, data):
    #     # TODO: utilizing data.name and data.distance
    #     with self.lock_parent_detected_target:
    #         if data.region == DetectedTarget.ROBOT:
    #             self.last_parent_detected_region = 'robot'
    #         elif data.region == DetectedTarget.CHILD:
    #             self.last_parent_detected_region = 'child'
    #         elif data.region == DetectedTarget.SCREEN:
    #             self.last_parent_detected_region = 'screen'
    #         elif data.region == DetectedTarget.OUTSIDE:
    #             self.last_parent_detected_region = 'outside'
    #         elif data.region == DetectedTarget.NONE:
    #             self.last_parent_detected_region = 'no detection'
    #         # rospy.loginfo("current parent attention target: "+ self.last_parent_detected_region)
    #     self.parent_attention_target_pub.publish(self.last_parent_detected_region)


    def system_state_callback(self, data):
        _sys_state = data.system_state
        if _sys_state == SystemState.SYSTEM_DOWN:
            self._f_system_down = True


    def send_robot_command(self, cmd, prop):
        # TODO: robot translator should be parsing lookat-user and lookat-xyz
        msg = RobotCommand()
        # add header
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.command = cmd
        msg.properties = prop
        # send message
        self.robot_command_pub.publish(msg)


um = user_monitor()
um.run()
