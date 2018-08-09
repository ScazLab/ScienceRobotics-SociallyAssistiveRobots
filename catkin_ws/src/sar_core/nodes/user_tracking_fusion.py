#!/usr/bin/env python
import threading

import rospy

import os

from std_msgs.msg import String
from geometry_msgs.msg import Vector3
from clm_ros_wrapper.msg import VectorWithCertainty, GazePointAndDirection, VectorsWithCertainty, GazePointsAndDirections, VectorsWithCertaintyWithGazePointsAndDirections, Assessment
from sar_core.msg import SystemState

child = 1;
parent = 2;

class user_tracking_fusion(object):
    #CLM_CERTAINTY_THRESHOLD = 0.5

    lock_head_position_and_gaze_point_left_cam = threading.Lock()
    lock_right_cam_user_head_position = threading.Lock()
    lock_left_cam_user_head_position = threading.Lock()
    lock_right_cam_user_gaze_point = threading.Lock()
    lock_left_cam_user_gaze_point = threading.Lock()
    # lock_parent_left_cam_user_gaze_point = threading.Lock()

    array_right_cam_head_x = []
    array_right_cam_head_y = []
    array_right_cam_head_z = []
    array_right_cam_certainty = []

    array_left_cam_head_x = []
    array_left_cam_head_y = []
    array_left_cam_head_z = []
    array_left_cam_head_certainty = []

    array_right_cam_gaze_point_and_direction = []
    array_left_cam_gaze_point_and_direction = []

    array_head_position_rf_x_left_cam = []
    array_head_position_rf_y_left_cam = []
    array_head_position_rf_z_left_cam = []
    array_head_position_rf_certainty_left_cam = []
    array_head_position_wf_x_left_cam = []
    array_head_position_wf_y_left_cam = []
    array_head_position_wf_z_left_cam = []
    array_gazes_left_cam = []


    last_user_head_x = 1000.0 #mm
    last_user_head_y = 0.0 #mm
    last_user_head_z = 1000.0 #mm
    last_user_head_certainty = 0.0
    last_gaze_point_certainty = 0.0
    last_gaze_point_wf = Vector3()
    last_head_position_wf = Vector3()
    last_hfv_wf = Vector3()

    parent_last_gaze_point_certainty = 0.0
    parent_last_gaze_point_wf = Vector3()
    parent_last_head_position_wf = Vector3()
    parent_last_hfv_wf = Vector3()

    last_right_cam_user_head_x = 1000.0 #mm
    last_right_cam_user_head_y = 0.0 #mm
    last_right_cam_user_head_z = 1000.0 #mm
    last_right_cam_user_head_certainty = 0.0
    last_right_cam_gaze_point_and_direction = GazePointAndDirection()

    last_left_cam_user_head_x = 1000.0 #mm
    last_left_cam_user_head_y = 0.0 #mm
    last_left_cam_user_head_z = 1000.0 #mm
    last_left_cam_user_head_certainty = 0.0
    last_left_cam_gaze_point_and_direction = GazePointAndDirection()


    parent_last_left_cam_user_head_x = 1000.0 #mm
    parent_last_left_cam_user_head_y = 0.0
    parent_last_left_cam_user_head_z = 1000.0 #mm
    parent_last_left_cam_user_head_certainty = 0.0 #mm
    parent_last_left_cam_gaze_point_and_direction = GazePointAndDirection()


    def __init__(self):
        rospy.init_node('user_tracking_fusion', anonymous=True)

        rospy.Subscriber("/sar/perception/left_cam/clm_ros_wrapper_0/head_position_and_eye_gaze", VectorsWithCertaintyWithGazePointsAndDirections, self.user_head_position_with_gaze_point_callback_left_cam)
        #rospy.Subscriber("/sar/perception/right_cam/clm_ros_wrapper_1/head_position_and_eye_gaze", VectorsWithCertaintyWithGazePointsAndDirections, self.user_head_position_with_gaze_point_callback_right_cam)

        #rospy.Subscriber("/sar/perception/right_cam/clm_ros_wrapper_1/head_position_rf", VectorWithCertainty, self.user_head_position_right_cam_callback)
        #rospy.Subscriber("/sar/perception/left_cam/clm_ros_wrapper_2/head_position_rf", VectorWithCertainty, self.user_head_position_left_cam_callback)

        #rospy.Subscriber("/sar/perception/right_cam/clm_ros_wrapper_1/gaze_point_and_direction", GazePointAndDirection, self.gaze_point_right_cam_callback)
        #rospy.Subscriber("/sar/perception/left_cam/clm_ros_wrapper_2/gaze_point_and_direction", GazePointAndDirection, self.gaze_point_left_cam_callback)

        self.fused_user_head_rf_pub = rospy.Publisher('/sar/perception/head_position_rf', VectorWithCertainty, queue_size=10)
        self.fused_gaze_point_wf_pub = rospy.Publisher('/sar/perception/gaze_point_and_direction_wf', GazePointsAndDirections, queue_size=10)


        rospy.Subscriber("/sar/system/state", SystemState, self.system_state_callback)

        rospy.Subscriber("/sar/perception/left_cam/clm_ros_wrapper_0/assessment", Assessment, self.assessment_callback)

        self.is_assessing = False
        self.total_num = 0
        self.correct_num = 0

        self._f_system_down = False

        self.fusion_mode = rospy.get_param('~flag_fusion_mode') #left, right, both

        self.rate = rospy.Rate(10) # 10hz 100ms


    def run(self):
        while not rospy.is_shutdown():
            if self._f_system_down:
                break
            # link left and right cam faces and determine which one of the faces is the chld
            self.determine_child_head_position_and_gaze()
            # update user head position in rf
            self.fuse_head_position_rf()
            # update user gaze point and direction in wf
            # self.fuse_gaze_point_wf()
            self.fuse_gaze_point_wf_2()
            # self.fuse_parent_gaze_point_wf_2()
            self.rate.sleep()


    def system_state_callback(self, data):
        _sys_state = data.system_state
        if _sys_state == SystemState.SYSTEM_DOWN:
            self._f_system_down = True

    def assessment_callback(self, data):
        if data.task != data.IDENTIFICATION:
            return

        if data.state == data.START:
            self.total_num = 0
            self.correct_num = 0
            if data.PARENT_MOM == data.parent_role:
                parent_role = "mom/";
            elif data.PARENT_DAD == data.parent_role:
                parent_role = "dad/";
            directory = '/home/sar/face_analyzer_assessment/both_faces/' + parent_role
            if not os.path.exists(directory):
                os.makedirs(directory)
            self.assessment_file = open(directory + 'identity.txt', 'w')
            self.is_assessing = True

        elif data.state == data.END:
            self.is_assessing = False
            self.assessment_file.write("accuracy\n")
            if 0 == self.total_num:
                self.assessment_file.write("0")
            else:
                self.assessment_file.write(str(float(self.correct_num) / float(self.total_num)))
            self.assessment_file.close()

    def determine_if_heads_are_in_similar_space(face_1_x, face_1_y, face_1_z, face_2_x, face_2_y, face_2_z):
        if (face_1_x * .90 < face_2_x) and (face_1_x * 1.10 > face_2_x) and \
            (face_1_y * .90 < face_2_y) and (face_1_y * 1.10 > face_2_y) and \
            (face_1_z * .90 < face_2_z) and (face_1_z * 1.10 > face_2_z):
            return 1
        else:
            return 0

    def determine_closer_head(compare_x, compare_y, compare_z, to_compare_0_x, to_compare_0_y, to_compare_0_z, to_compare_1_x, to_compare_1_y, to_compare_1_z):
        compare_to_0 = (compare_x - to_compare_0_x)**2 + (compare_y - to_compare_0_y)**2 + (compare_z - to_compare_0_z)**2
        compare_to_1 = (compare_x - to_compare_1_x)**2 + (compare_y - to_compare_1_y)**2 + (compare_z - to_compare_1_z)**2
        if(compare_to_0 > compare_to_1):
            return 1
        else:
            return 0

    def set_head_position_and_gazes_left_cam(self, data):
        self.last_left_cam_user_head_x = self.array_head_position_rf_x_left_cam[data]
        self.last_left_cam_user_head_y = self.array_head_position_rf_y_left_cam[data]
        self.last_left_cam_user_head_z = self.array_head_position_rf_z_left_cam[data]
        self.last_left_cam_user_head_certainty = self.array_head_position_rf_certainty_left_cam[data]
        self.last_left_cam_gaze_point_and_direction.gaze_point = self.array_gazes_left_cam[data].gaze_point
        self.last_left_cam_gaze_point_and_direction.head_position = self.array_gazes_left_cam[data].head_position
        self.last_left_cam_gaze_point_and_direction.hfv = self.array_gazes_left_cam[data].hfv
        self.last_left_cam_gaze_point_and_direction.certainty = self.array_gazes_left_cam[data].certainty
        self.last_left_cam_gaze_point_and_direction.role_confidence = self.array_gazes_left_cam[data].role_confidence

    def set_parent_head_position_and_gazes_left_cam(self, data):
        self.parent_last_left_cam_user_head_x = self.array_head_position_rf_x_left_cam[data]
        self.parent_last_left_cam_user_head_y = self.array_head_position_rf_y_left_cam[data]
        self.parent_last_left_cam_user_head_z = self.array_head_position_rf_z_left_cam[data]
        self.parent_last_left_cam_user_head_certainty = self.array_head_position_rf_certainty_left_cam[data]
        self.parent_last_left_cam_gaze_point_and_direction.gaze_point = self.array_gazes_left_cam[data].gaze_point
        self.parent_last_left_cam_gaze_point_and_direction.head_position = self.array_gazes_left_cam[data].head_position
        self.parent_last_left_cam_gaze_point_and_direction.hfv = self.array_gazes_left_cam[data].hfv
        self.parent_last_left_cam_gaze_point_and_direction.certainty = self.array_gazes_left_cam[data].certainty
        self.parent_last_left_cam_gaze_point_and_direction.role_confidence = self.array_gazes_left_cam[data].role_confidence        

    def set_left_cam_head_position_and_gazes_to_null(self):
        self.last_left_cam_user_head_x = 0.0
        self.last_left_cam_user_head_y = 0.0
        self.last_left_cam_user_head_z = 0.0
        self.last_left_cam_user_head_certainty = 0.0
        self.last_left_cam_gaze_point_and_direction.gaze_point = Vector3(0,0,0)
        self.last_left_cam_gaze_point_and_direction.head_position = Vector3(0,0,0)
        self.last_left_cam_gaze_point_and_direction.hfv = Vector3(0,0,0)
        self.last_left_cam_gaze_point_and_direction.certainty = 0.0
        self.last_left_cam_gaze_point_and_direction.role_confidence = 0.0

    def set_parent_left_cam_head_position_and_gazes_to_null(self):
        self.parent_last_left_cam_user_head_x = 0.0
        self.parent_last_left_cam_user_head_y = 0.0
        self.parent_last_left_cam_user_head_z = 0.0
        self.parent_last_left_cam_user_head_certainty = 0.0
        self.parent_last_left_cam_gaze_point_and_direction.gaze_point = Vector3(0,0,0)
        self.parent_last_left_cam_gaze_point_and_direction.head_position = Vector3(0,0,0)
        self.parent_last_left_cam_gaze_point_and_direction.hfv = Vector3(0,0,0)
        self.parent_last_left_cam_gaze_point_and_direction.certainty = 0.0
        self.parent_last_left_cam_gaze_point_and_direction.role_confidence = 0.0

    def determine_child_head_position_and_gaze(self):
        #Hardcoded for 2 faces
        #if 2 faces, take left one, if face between 0-500 in wf
        face_1_index = -1
        face_2_index = -1
        face_detected = 0

        # preprocess the faces
        # if only one face with confidence, then save that index face_1_index
        # if receive two faces with confidence, then save the left one to face_1_index, and save the right one to face_2_index
        if(len(self.array_gazes_left_cam) == 1):
            if (self.array_gazes_left_cam[0].certainty != 0.0 and self.array_gazes_left_cam[0].role != 0): # the face is not valid if the role is 0, which is the initialize value
                face_1_index = 0
                face_detected += 1
        elif(len(self.array_gazes_left_cam) == 2):
            if (self.array_gazes_left_cam[0].certainty != 0.0 and self.array_gazes_left_cam[0].role != 0):
                face_detected += 1
                face_1_index = 0
            if (self.array_gazes_left_cam[1].certainty != 0.0 and self.array_gazes_left_cam[1].role != 0):
                face_detected += 1
                if (2 == face_detected):
                    if (self.array_head_position_wf_x_left_cam[0] < self.array_head_position_wf_x_left_cam[1]):
                        face_2_index = 1
                    else:
                        face_1_index = 1
                        face_2_index = 0
                else:
                    face_1_index = 1

        face_string = "not set"
        if (0 == face_detected): # no face detected
            self.set_left_cam_head_position_and_gazes_to_null() # set no child face
            self.set_parent_left_cam_head_position_and_gazes_to_null() # set no parent face
            face_string = "no faces"
        elif (1 == face_detected): # detect one face
            if (self.array_gazes_left_cam[face_1_index].role == child): # this face belongs to child
                self.set_head_position_and_gazes_left_cam(face_1_index) # set child face
                self.set_parent_left_cam_head_position_and_gazes_to_null() # set no parent face
                face_string = "child"
            elif (self.array_gazes_left_cam[face_1_index].role == parent): # this face belongs to parent
                self.set_left_cam_head_position_and_gazes_to_null() # set no child face
                self.set_parent_head_position_and_gazes_left_cam(face_1_index) # set parent face
                face_string = "parent"
            else: # this face belongs to others, then use the position
                if (self.array_head_position_wf_x_left_cam[face_1_index] <= 0.5): # value in meter, in child zone
                    self.set_head_position_and_gazes_left_cam(face_1_index) # set child face
                    self.set_parent_left_cam_head_position_and_gazes_to_null() # set no parent face
                    face_string = "child"
                elif (self.array_head_position_wf_x_left_cam[face_1_index] <= 1.1): # value in meter, in parent zone
                    self.set_left_cam_head_position_and_gazes_to_null() # set no child face
                    self.set_parent_head_position_and_gazes_left_cam(face_1_index) # set parent face
                    face_string = "parent"
                else:
                    self.set_left_cam_head_position_and_gazes_to_null()
                    self.set_parent_left_cam_head_position_and_gazes_to_null()
                    face_string = "other"
        elif (2 == face_detected): # detect two faces
            if (self.array_gazes_left_cam[face_1_index].role == self.array_gazes_left_cam[face_2_index].role): # both have the same role
                if (self.array_gazes_left_cam[face_1_index].role == child): # both faces belongs to child
                    if (self.array_gazes_left_cam[face_1_index].role_confidence <= self.array_gazes_left_cam[face_2_index].role_confidence): # confidence is the distance
                        self.set_head_position_and_gazes_left_cam(face_1_index)
                        if (self.array_head_position_wf_x_left_cam[face_2_index] <= 1.1): # the other face within the parent range
                            self.set_parent_head_position_and_gazes_left_cam(face_2_index)
                            face_string = "child parent"
                        else:
                            self.set_parent_left_cam_head_position_and_gazes_to_null()
                            face_string = "child other"
                    else: # the face at the right is the child
                        self.set_head_position_and_gazes_left_cam(face_2_index)
                        self.set_parent_left_cam_head_position_and_gazes_to_null()
                        face_string = "other child"
                elif (self.array_gazes_left_cam[face_1_index].role == parent): # both faces belong to parent
                    if (self.array_gazes_left_cam[face_2_index].role_confidence <= self.array_gazes_left_cam[face_1_index].role_confidence): # confidence is the distance
                        self.set_parent_head_position_and_gazes_left_cam(face_2_index)
                        if (self.array_head_position_wf_x_left_cam[face_1_index] <= 0.5): # the other face within the parent range
                            self.set_head_position_and_gazes_left_cam(face_1_index)
                            face_string = "child parent"
                        else: # the other face is not within the child's range
                            self.set_left_cam_head_position_and_gazes_to_null()
                            face_string = "other parent"
                    else: # the face at the left is the parent
                        self.set_parent_head_position_and_gazes_left_cam(face_1_index)
                        self.set_left_cam_head_position_and_gazes_to_null()
                        face_string = "parent other"
                else: # both faces belong to others, purely based on position
                    if (self.array_head_position_wf_x_left_cam[face_1_index] <= 0.5):
                        self.set_head_position_and_gazes_left_cam(face_1_index)
                        face_string = "child"
                    else:
                        self.set_left_cam_head_position_and_gazes_to_null()
                        face_string = "other"

                    if (self.array_head_position_wf_x_left_cam[face_2_index] <= 1.1):
                        self.set_parent_head_position_and_gazes_left_cam(face_2_index)
                        if face_string != "" :
                            face_string += " "
                        face_string += "parent"
                    else:
                        self.set_parent_left_cam_head_position_and_gazes_to_null()
                        if face_string != "" :
                            face_string += " "
                        face_string += "other"
            elif (self.array_gazes_left_cam[face_1_index].role == child and self.array_gazes_left_cam[face_2_index].role == parent): # left is child and right is parent
                self.set_head_position_and_gazes_left_cam(face_1_index)
                self.set_parent_head_position_and_gazes_left_cam(face_2_index)
                face_string = "child parent"
            elif (self.array_gazes_left_cam[face_2_index].role == child and self.array_gazes_left_cam[face_1_index].role == parent): # CM: left is parent and right is child
                self.set_head_position_and_gazes_left_cam(face_2_index)
                self.set_parent_head_position_and_gazes_left_cam(face_1_index)
                face_string = "parent child"
            else: # other and child/parent
                if (self.array_gazes_left_cam[face_1_index].role == child): # child + other
                    self.set_head_position_and_gazes_left_cam(face_1_index)
                    if (self.array_head_position_wf_x_left_cam[face_2_index] <= 1.1): # if the other face is within the parent boundary
                        self.set_parent_head_position_and_gazes_left_cam(face_2_index)
                        face_string = "child parent"
                    else:
                        self.set_parent_left_cam_head_position_and_gazes_to_null()
                        face_string = "child other"
                elif (self.array_gazes_left_cam[face_2_index].role == parent): # other + parent
                    if (self.array_head_position_wf_x_left_cam[face_1_index] <= 0.5): # if the other is in within the child boundary
                        self.set_head_position_and_gazes_left_cam(face_1_index)
                        face_string = "child parent"
                    else:
                        self.set_left_cam_head_position_and_gazes_to_null()
                        face_string = "other parent"
                elif (self.array_gazes_left_cam[face_2_index].role == child): # other + child
                    self.set_head_position_and_gazes_left_cam(face_2_index)
                    self.set_parent_left_cam_head_position_and_gazes_to_null() # no parent
                    face_string = "other child"
                elif (self.array_gazes_left_cam[face_1_index].role == parent): # parent + other
                    self.set_parent_head_position_and_gazes_left_cam(face_1_index)
                    self.set_left_cam_head_position_and_gazes_to_null()
                    face_string = "parent other"
        # print face_string
        if self.is_assessing:
            self.total_num += 1
            self.assessment_file.write(face_string + "\n");
            if face_string == "child parent":
                self.correct_num += 1

    def fuse_head_position_rf(self):
        if self.fusion_mode == 'both':
            with self.lock_right_cam_user_head_position and self.lock_left_cam_user_head_position:
                if (self.last_right_cam_user_head_certainty == 0.0) and (self.last_left_cam_user_head_certainty == 0.0):
                    self.last_user_head_x = 0.0
                    self.last_user_head_y = 0.0
                    self.last_user_head_z = 0.0
                    self.last_user_head_certainty = 0.0
                elif (self.last_right_cam_user_head_certainty == 0.0):
                    self.last_user_head_x = self.last_left_cam_user_head_x
                    self.last_user_head_y = self.last_left_cam_user_head_y
                    self.last_user_head_z = self.last_left_cam_user_head_z
                    self.last_user_head_certainty = self.last_left_cam_user_head_certainty
                elif (self.last_left_cam_user_head_certainty == 0.0):
                    self.last_user_head_x = self.last_right_cam_user_head_x
                    self.last_user_head_y = self.last_right_cam_user_head_y
                    self.last_user_head_z = self.last_right_cam_user_head_z
                    self.last_user_head_certainty = self.last_right_cam_user_head_certainty
                else:
                    _sum = self.last_right_cam_user_head_certainty + self.last_left_cam_user_head_certainty
                    self.last_user_head_x = (self.last_right_cam_user_head_certainty * self.last_right_cam_user_head_x +
                        self.last_left_cam_user_head_certainty * self.last_left_cam_user_head_x) / _sum
                    self.last_user_head_y = (self.last_right_cam_user_head_certainty * self.last_right_cam_user_head_y +
                        self.last_left_cam_user_head_certainty * self.last_left_cam_user_head_y) / _sum
                    self.last_user_head_z = (self.last_right_cam_user_head_certainty * self.last_right_cam_user_head_z +
                        self.last_left_cam_user_head_certainty * self.last_left_cam_user_head_z) / _sum
                    self.last_user_head_certainty = (0.5 * self.last_right_cam_user_head_certainty +
                        0.5 * self.last_left_cam_user_head_certainty)
        elif self.fusion_mode == 'right':
            with self.lock_right_cam_user_head_position:
                self.last_user_head_x = self.last_right_cam_user_head_x
                self.last_user_head_y = self.last_right_cam_user_head_y
                self.last_user_head_z = self.last_right_cam_user_head_z
                self.last_user_head_certainty = self.last_right_cam_user_head_certainty
        elif self.fusion_mode == 'left':
            with self.lock_left_cam_user_head_position:
                self.last_user_head_x = self.last_left_cam_user_head_x
                self.last_user_head_y = self.last_left_cam_user_head_y
                self.last_user_head_z = self.last_left_cam_user_head_z
                self.last_user_head_certainty = self.last_left_cam_user_head_certainty

        # publish head position in rf
        vwc = VectorWithCertainty()
        vwc.certainty = self.last_user_head_certainty
        _vec = Vector3()
        _vec.x = self.last_user_head_x
        _vec.y = self.last_user_head_y
        _vec.z = self.last_user_head_z
        vwc.position = _vec
        self.fused_user_head_rf_pub.publish(vwc)


    def fuse_gaze_point_wf(self):
        if self.fusion_mode == 'both':
            with self.lock_right_cam_user_gaze_point and self.lock_left_cam_user_gaze_point:
                if (self.last_right_cam_gaze_point_and_direction.certainty == 0.0) and (self.last_left_cam_gaze_point_and_direction.certainty == 0.0):
                    self.last_gaze_point_wf = Vector3(0,0,0)
                    self.last_head_position_wf = Vector3(0,0,0)
                    self.last_hfv_wf = Vector3(0,0,0)
                    self.last_gaze_point_certainty = 0.0
                elif (self.last_right_cam_user_head_certainty == 0.0):
                    self.last_gaze_point_wf = self.last_left_cam_gaze_point_and_direction.gaze_point
                    self.last_head_position_wf = self.last_left_cam_gaze_point_and_direction.head_position
                    self.last_hfv_wf = self.last_left_cam_gaze_point_and_direction.hfv
                    self.last_gaze_point_certainty = self.last_left_cam_gaze_point_and_direction.certainty
                elif (self.last_left_cam_user_head_certainty == 0.0):
                    self.last_gaze_point_wf = self.last_right_cam_gaze_point_and_direction.gaze_point
                    self.last_head_position_wf = self.last_right_cam_gaze_point_and_direction.head_position
                    self.last_hfv_wf = self.last_right_cam_gaze_point_and_direction.hfv
                    self.last_gaze_point_certainty = self.last_right_cam_gaze_point_and_direction.certainty
                else:
                    _fused_vec3 = Vector3()
                    # gaze point
                    _sum = self.last_right_cam_gaze_point_and_direction.certainty + self.last_left_cam_gaze_point_and_direction.certainty
                    _fused_vec3.x = (self.last_right_cam_gaze_point_and_direction.certainty * self.last_right_cam_gaze_point_and_direction.gaze_point.x +
                        self.last_left_cam_gaze_point_and_direction.certainty * self.last_left_cam_gaze_point_and_direction.gaze_point.x) / _sum
                    _fused_vec3.y = (self.last_right_cam_gaze_point_and_direction.certainty * self.last_right_cam_gaze_point_and_direction.gaze_point.y +
                        self.last_left_cam_gaze_point_and_direction.certainty * self.last_left_cam_gaze_point_and_direction.gaze_point.y) / _sum
                    _fused_vec3.z = (self.last_right_cam_gaze_point_and_direction.certainty * self.last_right_cam_gaze_point_and_direction.gaze_point.z +
                        self.last_left_cam_gaze_point_and_direction.certainty * self.last_left_cam_gaze_point_and_direction.gaze_point.z) / _sum
                    self.last_gaze_point_wf = _fused_vec3
                    # head position in wf
                    _fused_vec3.x = (self.last_right_cam_gaze_point_and_direction.certainty * self.last_right_cam_gaze_point_and_direction.head_position.x +
                        self.last_left_cam_gaze_point_and_direction.certainty * self.last_left_cam_gaze_point_and_direction.head_position.x) / _sum
                    _fused_vec3.y = (self.last_right_cam_gaze_point_and_direction.certainty * self.last_right_cam_gaze_point_and_direction.head_position.y +
                        self.last_left_cam_gaze_point_and_direction.certainty * self.last_left_cam_gaze_point_and_direction.head_position.y) / _sum
                    _fused_vec3.z = (self.last_right_cam_gaze_point_and_direction.certainty * self.last_right_cam_gaze_point_and_direction.head_position.z +
                        self.last_left_cam_gaze_point_and_direction.certainty * self.last_left_cam_gaze_point_and_direction.head_position.z) / _sum
                    self.last_head_position_wf = _fused_vec3
                    # hfv
                    _fused_vec3.x = (self.last_right_cam_gaze_point_and_direction.certainty * self.last_right_cam_gaze_point_and_direction.hfv.x +
                        self.last_left_cam_gaze_point_and_direction.certainty * self.last_left_cam_gaze_point_and_direction.hfv.x) / _sum
                    _fused_vec3.y = (self.last_right_cam_gaze_point_and_direction.certainty * self.last_right_cam_gaze_point_and_direction.hfv.y +
                        self.last_left_cam_gaze_point_and_direction.certainty * self.last_left_cam_gaze_point_and_direction.hfv.y) / _sum
                    _fused_vec3.z = (self.last_right_cam_gaze_point_and_direction.certainty * self.last_right_cam_gaze_point_and_direction.hfv.z +
                        self.last_left_cam_gaze_point_and_direction.certainty * self.last_left_cam_gaze_point_and_direction.hfv.z) / _sum
                    self.last_hfv_wf = _fused_vec3
                    # certainty
                    self.last_gaze_point_certainty = (0.5 * self.last_right_cam_gaze_point_and_direction.certainty +
                        0.5 * self.last_left_cam_gaze_point_and_direction.certainty)
        elif self.fusion_mode == 'right':
            with self.lock_right_cam_user_gaze_point:
                self.last_gaze_point_wf = self.last_right_cam_gaze_point_and_direction.gaze_point
                self.last_head_position_wf = self.last_right_cam_gaze_point_and_direction.head_position
                self.last_hfv_wf = self.last_right_cam_gaze_point_and_direction.hfv
                self.last_gaze_point_certainty = self.last_right_cam_gaze_point_and_direction.certainty
        elif self.fusion_mode == 'left':
            with self.lock_left_cam_user_gaze_point:
                self.last_gaze_point_wf = self.last_left_cam_gaze_point_and_direction.gaze_point
                self.last_head_position_wf = self.last_left_cam_gaze_point_and_direction.head_position
                self.last_hfv_wf = self.last_left_cam_gaze_point_and_direction.hfv
                self.last_gaze_point_certainty = self.last_left_cam_gaze_point_and_direction.certainty

        # publish gaze point and direction in wf
        gpd = GazePointAndDirection()
        # certainties for gaze point and head position should be the same
        gpd.certainty = self.last_gaze_point_certainty
        gpd.gaze_point = self.last_gaze_point_wf
        gpd.head_position = self.last_head_position_wf
        gpd.hfv = self.last_hfv_wf
        gpd.role_confidence = self.last_left_cam_gaze_point_and_direction.role_confidence
        gpd.role = gpd.CHILD_ROLE

        self.fused_gaze_point_wf_pub.publish(gpd)

    # pick the one with higher certainty
    def fuse_gaze_point_wf_2(self):
        if self.fusion_mode == 'both':
            with self.lock_right_cam_user_gaze_point and self.lock_left_cam_user_gaze_point:
                if (self.last_right_cam_gaze_point_and_direction.certainty == 0.0) and (self.last_left_cam_gaze_point_and_direction.certainty == 0.0):
                    self.last_gaze_point_wf = Vector3(0,0,0)
                    self.last_head_position_wf = Vector3(0,0,0)
                    self.last_hfv_wf = Vector3(0,0,0)
                    self.last_gaze_point_certainty = 0.0
                elif (self.last_right_cam_user_head_certainty == 0.0 or self.last_right_cam_gaze_point_and_direction.certainty <= self.last_left_cam_gaze_point_and_direction.certainty):
                    self.last_gaze_point_wf = self.last_left_cam_gaze_point_and_direction.gaze_point
                    self.last_head_position_wf = self.last_left_cam_gaze_point_and_direction.head_position
                    self.last_hfv_wf = self.last_left_cam_gaze_point_and_direction.hfv
                    self.last_gaze_point_certainty = self.last_left_cam_gaze_point_and_direction.certainty
                elif (self.last_left_cam_user_head_certainty == 0.0 or self.last_right_cam_gaze_point_and_direction.certainty > self.last_left_cam_gaze_point_and_direction.certainty):
                    self.last_gaze_point_wf = self.last_right_cam_gaze_point_and_direction.gaze_point
                    self.last_head_position_wf = self.last_right_cam_gaze_point_and_direction.head_position
                    self.last_hfv_wf = self.last_right_cam_gaze_point_and_direction.hfv
                    self.last_gaze_point_certainty = self.last_right_cam_gaze_point_and_direction.certainty
        elif self.fusion_mode == 'right':
            with self.lock_right_cam_user_gaze_point:
                self.last_gaze_point_wf = self.last_right_cam_gaze_point_and_direction.gaze_point
                self.last_head_position_wf = self.last_right_cam_gaze_point_and_direction.head_position
                self.last_hfv_wf = self.last_right_cam_gaze_point_and_direction.hfv
                self.last_gaze_point_certainty = self.last_right_cam_gaze_point_and_direction.certainty
        elif self.fusion_mode == 'left':
            with self.lock_left_cam_user_gaze_point:
                self.last_gaze_point_wf = self.last_left_cam_gaze_point_and_direction.gaze_point
                self.last_head_position_wf = self.last_left_cam_gaze_point_and_direction.head_position
                self.last_hfv_wf = self.last_left_cam_gaze_point_and_direction.hfv
                self.last_gaze_point_certainty = self.last_left_cam_gaze_point_and_direction.certainty
                self.parent_last_gaze_point_wf = self.parent_last_left_cam_gaze_point_and_direction.gaze_point
                self.parent_last_head_position_wf = self.parent_last_left_cam_gaze_point_and_direction.head_position
                self.parent_last_hfv_wf = self.parent_last_left_cam_gaze_point_and_direction.hfv
                self.parent_last_gaze_point_certainty = self.parent_last_left_cam_gaze_point_and_direction.certainty

        # publish gaze point and direction in wf
        gpd = GazePointAndDirection()
        # certainties for gaze point and head position should be the same
        gpd.certainty = self.last_gaze_point_certainty
        gpd.gaze_point = self.last_gaze_point_wf
        gpd.head_position = self.last_head_position_wf
        gpd.hfv = self.last_hfv_wf
        gpd.role_confidence = self.last_left_cam_gaze_point_and_direction.role_confidence
        gpd.role = gpd.CHILD_ROLE

        # publish gaze point and direction in wf
        parent_gpd = GazePointAndDirection()
        # certainties for gaze point and head position should be the same
        parent_gpd.certainty = self.parent_last_gaze_point_certainty
        parent_gpd.gaze_point = self.parent_last_gaze_point_wf
        parent_gpd.head_position = self.parent_last_head_position_wf
        parent_gpd.hfv = self.parent_last_hfv_wf
        parent_gpd.role_confidence = self.parent_last_left_cam_gaze_point_and_direction.role_confidence
        parent_gpd.role = parent_gpd.PARENT_ROLE

        gpd_message = GazePointsAndDirections()
        gpd_message.gazes.append(gpd)
        gpd_message.gazes.append(parent_gpd)

        #print "[py] gpd certainty %2f gaze point x %2f gaze point y %2f gaze point z %2f" % (gpd.certainty, gpd.gaze_point.x, gpd.gaze_point.y, gpd.gaze_point.z)
        self.fused_gaze_point_wf_pub.publish(gpd_message)

    # def fuse_parent_gaze_point_wf_2(self):
    #     if self.fusion_mode == 'left':
    #         with self.lock_parent_left_cam_user_gaze_point:
    #             self.parent_last_gaze_point_wf = self.parent_last_left_cam_gaze_point_and_direction.gaze_point
    #             self.parent_last_head_position_wf = self.parent_last_left_cam_gaze_point_and_direction.head_position
    #             self.parent_last_hfv_wf = self.parent_last_left_cam_gaze_point_and_direction.hfv
    #             self.parent_last_gaze_point_certainty = self.parent_last_left_cam_gaze_point_and_direction.certainty

    #     # publish gaze point and direction in wf
    #     gpd = GazePointAndDirection()
    #     # certainties for gaze point and head position should be the same
    #     gpd.certainty = self.parent_last_gaze_point_certainty
    #     gpd.gaze_point = self.parent_last_gaze_point_wf
    #     gpd.head_position = self.parent_last_head_position_wf
    #     gpd.hfv = self.parent_last_hfv_wf
        
    #     # print "publish parent"
    #     self.fused_parent_gaze_point_wf_pub.publish(gpd)

    def user_head_position_with_gaze_point_callback_left_cam(self, data):
        with self.lock_head_position_and_gaze_point_left_cam:
            self.array_head_position_rf_x_left_cam = []
            self.array_head_position_rf_y_left_cam = []
            self.array_head_position_rf_z_left_cam = []
            self.array_head_position_rf_certainty_left_cam = []
            self.array_head_position_wf_x_left_cam = []
            self.array_head_position_wf_y_left_cam = []
            self.array_head_position_wf_z_left_cam = []
            self.array_gazes_left_cam = []

            for vectors_loop in data.vectors_rf:
                self.array_head_position_rf_x_left_cam.append(vectors_loop.position.x / 1000.0) #in meters
                self.array_head_position_rf_y_left_cam.append(vectors_loop.position.y / 1000.0) #in meters
                #z + 120 for compensating the distance between nose and eyes
                self.array_head_position_rf_z_left_cam.append((vectors_loop.position.z + 120.0) / 1000.0) #in meters
                self.array_head_position_rf_certainty_left_cam.append(vectors_loop.certainty)

            for vectors_loop in data.head_position_wf:
                self.array_head_position_wf_x_left_cam.append(vectors_loop.x / 1000.0) #in meters
                self.array_head_position_wf_y_left_cam.append(vectors_loop.y / 1000.0) #in meters
                #z + 120 for compensating the distance between nose and eyes
                self.array_head_position_wf_z_left_cam.append((vectors_loop.z + 120.0) / 1000.0) #in meters

            for gazes_loop in data.gazes:
                self.array_gazes_left_cam.append(GazePointAndDirection())
                self.array_gazes_left_cam[-1].certainty = gazes_loop.certainty
                self.array_gazes_left_cam[-1].gaze_point = gazes_loop.gaze_point
                self.array_gazes_left_cam[-1].head_position = gazes_loop.head_position
                self.array_gazes_left_cam[-1].hfv = gazes_loop.hfv
                self.array_gazes_left_cam[-1].role = gazes_loop.role
                self.array_gazes_left_cam[-1].role_confidence = gazes_loop.role_confidence

    #def user_head_position_right_cam_callback(self, data):
    #    with self.lock_right_cam_user_head_position:
    #        self.last_right_cam_user_head_x = data.position.x / 1000.0 #in meters
    #        self.last_right_cam_user_head_y = data.position.y / 1000.0 #in meters
    #        #z + 120 for compensating the distance between nose and eyes
    #        self.last_right_cam_user_head_z = (data.position.z + 120.0) / 1000.0 #in meters
    #        self.last_right_cam_user_head_certainty = data.certainty

    #def user_head_position_left_cam_callback(self, data):
    #    with self.lock_left_cam_user_head_position:
    #        self.last_left_cam_user_head_x = data.position.x / 1000.0 #in meters
    #        self.last_left_cam_user_head_y = data.position.y / 1000.0 #in meters
    #        #z + 120 for compensating the distance between nose and eyes
    #        self.last_left_cam_user_head_z = (data.position.z + 120.0) / 1000.0 #in meters
    #        self.last_left_cam_user_head_certainty = data.certainty

    #def gaze_point_right_cam_callback(self, data):
    #    with self.lock_right_cam_user_gaze_point:
    #        self.last_right_cam_gaze_point_and_direction.certainty = data.certainty
    #        self.last_right_cam_gaze_point_and_direction.gaze_point = data.gaze_point
    #        self.last_right_cam_gaze_point_and_direction.head_position = data.head_position
    #        self.last_right_cam_gaze_point_and_direction.hfv = data.hfv

    #def gaze_point_left_cam_callback(self, data):
    #    with self.lock_left_cam_user_gaze_point:
    #        self.last_left_cam_gaze_point_and_direction.certainty = data.certainty
    #        self.last_left_cam_gaze_point_and_direction.gaze_point = data.gaze_point
    #        self.last_left_cam_gaze_point_and_direction.head_position = data.head_position
    #        self.last_left_cam_gaze_point_and_direction.hfv = data.hfv

utf = user_tracking_fusion()
utf.run()
