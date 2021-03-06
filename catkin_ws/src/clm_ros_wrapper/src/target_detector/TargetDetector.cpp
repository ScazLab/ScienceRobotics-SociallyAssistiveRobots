
/*
This ros node subscribes to "/clm_ros_wrapper/gaze_point" and finds the target on the screen where this
gaze point falls into. It then publishes this information with publisher topic "/clm_ros_wrapper/detect_target"
*/
#include "CLM_core.h"
#include <std_msgs/String.h>

#include <fstream>
#include <sstream>

// #include <opencv2/core/core.hpp>
// #include <opencv2/highgui/highgui.hpp>
// #include <opencv2/imgproc/imgproc.hpp>

#include <Face_utils.h>
#include <FaceAnalyser.h>
#include <GazeEstimation.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <clm_ros_wrapper/ClmHeads.h>
#include <clm_ros_wrapper/ClmEyeGaze.h>
#include <clm_ros_wrapper/ClmFacialActionUnit.h>
#include <clm_ros_wrapper/Scene.h>
#include <clm_ros_wrapper/Object.h>
#include <clm_ros_wrapper/DetectedTarget.h>
#include <clm_ros_wrapper/DetectedTargets.h>
#include <clm_ros_wrapper/GazePointAndDirection.h>
#include <clm_ros_wrapper/GazePointsAndDirections.h>
#include <clm_ros_wrapper/Assessment.h>
#include <sar_core/SystemState.h>

#include <filesystem.hpp>
#include <filesystem/fstream.hpp>

#include <math.h>
#include <vector>

#include <tf/transform_datatypes.h>

#include <geometry_msgs/Vector3.h>
#include <limits>

#define max_num_objects 20


using namespace std;
using namespace boost::filesystem;

int screenAngleInDegrees;
float screenWidth;
float screenHeight;
float screenAngle;
float screenGap;
string _namespace;
std::ofstream assessment_child_file;
std::ofstream assessment_parent_file;
int assessment_child_total_num;
int assessment_child_total_detected_num;
int assessment_child_correct_num;
string assessment_child_correct_answer;
int assessment_parent_total_num;
int assessment_parent_total_detected_num;
int assessment_parent_correct_num;
string assessment_parent_correct_answer;

bool is_assessing;

int num_objects_on_screen, num_free_objects;

//the positions are loaded to these arrays
tf::Vector3 screen_reference_points_wf [max_num_objects];
tf::Vector3 free_objects_positions [max_num_objects];

//the objects' names are here with the same index as they have
//in the array for positions
std::string screen_reference_points_names [max_num_objects];
std::string free_objects_names [max_num_objects];

//to do in/out checks for the free objects
float free_object_radius;

float robot_position_wf_x, robot_position_wf_y, robot_position_wf_z;

ros::Publisher targets_publisher;
// ros::Publisher target_publisher;
// ros::Publisher parent_target_publisher;

using namespace std;
using std::vector;

// a set of virtual targets of the child in wf (mm)
vector<tf::Vector3> child_virtual_targets;

// a set of virtual targets of the parent in wf (mm)
vector<tf::Vector3> parent_virtual_targets;

float my_dot_product(tf::Vector3 a, tf::Vector3 b)// should be the same as dot
{
    return a.getX()*b.getX() + a.getY()*b.getY() + a.getZ()*b.getZ();
}

float my_vec_magn(tf::Vector3 v)// should be the same as length
{
    return  sqrt(v.getX()*v.getX() + v.getY()*v.getY() + v.getZ()*v.getZ());
}

tf::Vector3 my_scaling_vec(float scaler, tf::Vector3 v)// should be the same as *
{
    return tf::Vector3(v.getX()*scaler, v.getY()*scaler, v.getZ()*scaler);
}

bool is_point_inside_cone(tf::Vector3 top_point, tf::Vector3 direction_vec, float height, float radian, tf::Vector3 test_point, double &dist)
{
    // http://stackoverflow.com/questions/10768142/verify-if-point-is-inside-a-cone-in-3d-space
    // check 1:
    bool is_in_infinite_cone = false;
    tf::Vector3 apex_to_point = test_point - top_point;
    tf::Vector3 apex_to_bottom = my_scaling_vec(height, direction_vec);
    double cos_point_angle = my_dot_product(apex_to_point, apex_to_bottom)/my_vec_magn(apex_to_point)/my_vec_magn(apex_to_bottom);
    double sine_point_angle = sqrt(1 - cos_point_angle*cos_point_angle);
    dist = my_vec_magn(apex_to_point)*sine_point_angle; // vertical distance to the head direction
    is_in_infinite_cone = (cos_point_angle > cos(radian/2.0));//15 degrees in radian: 0.261799
    if(!is_in_infinite_cone) return false;

    //check 2:
    bool is_under_round_cap = false;
    double projected_h = my_dot_product(apex_to_point, apex_to_bottom)/my_vec_magn(apex_to_bottom);
    is_under_round_cap = (projected_h < my_vec_magn(apex_to_bottom));

    return is_under_round_cap;
}

/*
void gazepoint_callback(const clm_ros_wrapper::GazePointAndDirection::ConstPtr& msg)
{
    double detection_certainty = (*msg).certainty;
    tf::Vector3 gaze_point_wf, head_position_wf, hfv_wf;

    //checking if there is detection
    tf::vector3MsgToTF((*msg).gaze_point, gaze_point_wf);
    tf::vector3MsgToTF((*msg).head_position, head_position_wf);
    tf::vector3MsgToTF((*msg).hfv, hfv_wf);

    if (gaze_point_wf.isZero() && head_position_wf.isZero() && hfv_wf.isZero())
    {
        //means no detection
        clm_ros_wrapper::DetectedTarget target_no_detection;
        target_no_detection.certainty = detection_certainty;

        target_no_detection.name = "NO DETECTION";
        target_no_detection.distance = 0;
        target_no_detection.region = target_no_detection.NONE;

        target_publisher.publish(target_no_detection);
    }

    else
    {
        clm_ros_wrapper::DetectedTarget detected_target;
        detected_target.certainty = detection_certainty;

        int num_closest_object_on_screen = 0, num_closest_free_object = 0;

        // TODO: cmhuang: change this..
        // to make sure this callback happens after scene_callback
        if (num_objects_on_screen != 0 || num_free_objects != 0)
        {

            float closest_distance_screen = std::numeric_limits<double>::max();
            float closest_distance_free_object = std::numeric_limits<double>::max();

            // find the closet object displayed on the screen
            for (int i = 0; i<num_objects_on_screen; i++)
            {
                if (closest_distance_screen > gaze_point_wf.distance(screen_reference_points_wf[i]))
                {
                    closest_distance_screen = gaze_point_wf.distance(screen_reference_points_wf[i]);
                    num_closest_object_on_screen = i;
                }
            }

            //checking for the  boundaries of the screen in Z and X axes to see
            //if the gazepoint is outside of the screen
            //3 *screenGap on the bottom because the gap is bigger
            if ((3) * screenGap * sin(screenAngle) > gaze_point_wf.getZ() || (screenHeight  - screenGap)*sin(screenAngle) < gaze_point_wf.getZ()
                || gaze_point_wf.getX() > screenWidth / 2 - screenGap || gaze_point_wf.getX() < (-1) * screenWidth / 2 +  screenGap)
            {
                num_closest_object_on_screen = num_objects_on_screen;
                // the index num_closest_object_on_screen refers to outside -- is the object named "Outside"
                closest_distance_screen = std::numeric_limits<double>::max();
            }

            //USING THE LINE-POINT DISTANCE FORMULA TO FIND THE CLOSEST FREE OBJECT
            // To see the calculations in more depth: http://mathworld.wolfram.com/Point-LineDistance3-Dimensional.html
            // Explanation:
            // X_1: head_position_wf, X_2:randompoint_on_gazedirection X_0:free object's position

            tf::Vector3 randompoint_on_gazedirection = head_position_wf + 1000 * hfv_wf;

            //dummy zero vector because the function length is defined for quaternions only ???
            tf::Vector3 zero_vector = tf::Vector3(0,0,0);

            for (int i = 0; i < num_free_objects; i++)
            {
                tf::Vector3 diff_freeobj_headpos = free_objects_positions[i]-head_position_wf;
                tf::Vector3 diff_freeobj_rand = free_objects_positions[i]-randompoint_on_gazedirection;

                //using the formula from the link
                // float distance = zero_vector.distance(diff_freeobj_headpos.cross(diff_freeobj_rand))\
                //     /zero_vector.distance(randompoint_on_gazedirection - head_position_wf);
                float distance = zero_vector.distance(diff_freeobj_headpos.cross(diff_freeobj_rand))\
                    /zero_vector.distance(randompoint_on_gazedirection - head_position_wf);

                if (closest_distance_free_object > distance)
                {
                    closest_distance_free_object = distance;
                    num_closest_free_object = i;
                }
            }
            // inside/outside check for the closest free object
            // free_object_radius is the robot radius
            // TODO: change this parameter
            if (closest_distance_free_object > free_object_radius)
            {
                num_closest_free_object = num_free_objects;
                closest_distance_free_object = std::numeric_limits<double>::max();
                //setting it to "OUTSIDE"
            }

            // This part changes because of the new free object message type
            // if (screen_reference_points_names[num_closest_target].compare("robot")!=0)
            // {
            // // WHAT IF THE POINT IS OUTSIDE THE SCREEN
            // // do a check to see if the point is inside
            //     if ((-1)* screenGap > gazepoint.getZ() || screenHeight * sin(screenAngle)  + screenGap < gazepoint.getZ()
            //         || gazepoint.getX() > screenWidth / 2 + screenGap || gazepoint.getX() < (-1) * screenWidth / 2 - screenGap)
            //     {
            //         num_closest_target = num_objects;
            //     }
            // }
            // //this part should change in the next commits
            // // you should use the head location to estimate whether the kid is looking at the robot
            // else //num_closest_target is the index of the object named robot
            // {
            //     if (closest_distance > 3 * screenGap)
            //     {
            //         num_closest_target = num_objects;
            //     }
            // }


            if (screen_reference_points_names[num_closest_object_on_screen].compare("OUTSIDE") == 0
                && free_objects_names[num_closest_free_object].compare("OUTSIDE") == 0)
            {
                detected_target.name = "OUTSIDE";
                detected_target.distance = 0.0;
                detected_target.region = detected_target.OUTSIDE;
            }
            else
            {
                // checking between free objects and screen objects to find the closest
                if (closest_distance_free_object > closest_distance_screen)
                {
                    detected_target.name = screen_reference_points_names[num_closest_object_on_screen];
                    detected_target.distance = closest_distance_screen;
                    detected_target.region = detected_target.SCREEN;
                }
                else
                {
                    detected_target.name = free_objects_names[num_closest_free_object];
                    detected_target.distance = closest_distance_free_object;
                    if(detected_target.name == "robot"){
                        detected_target.region = detected_target.ROBOT;
                    }
                    else if(detected_target.name == "parent"){
                        detected_target.region = detected_target.PARENT;
                    }
                }
            }

            //target_publisher.publish(detected_target);
            //cout << endl << num_closest_target << endl << endl;
        }

        //clm_ros_wrapper::DetectedTarget detected_target;

        // if (num_closest_object_on_screen == num_objects_on_screen) // means the point is outside the screen
        // {
        //     detected_target.region = detected_target.OUTSIDE;
        // }
        // else
        // {
        //     detected_target.region = detected_target.SCREEN;
        // }

        // TODO: the robot position should come from the scene publisher?
        // cmhuang: I do not know why we need the following code
        // tf::Vector3 robot_position_tf = tf::Vector3(robot_position_wf_x, robot_position_wf_y, robot_position_wf_z);

        // tf::Vector3 randompoint_on_gazedirection = head_position_wf + 1000 * hfv_wf;

        // tf::Vector3 diff_robotpos_headpos = robot_position_tf-head_position_wf;
        // tf::Vector3 diff_robotpos_rand = robot_position_tf-randompoint_on_gazedirection;

        // //using the formula from the link
        // float distance = tf::Vector3(0,0,0).distance(diff_robotpos_headpos.cross(diff_robotpos_rand))/tf::Vector3(0,0,0).distance(randompoint_on_gazedirection - head_position_wf);
        // if (distance < free_object_radius)
        // {
        //     detected_target.region = detected_target.ROBOT;
        // }

        // else if (detected_target.region == detected_target.SCREEN)
        // {
        //     detected_target.region = detected_target.SCREEN;
        // }

        // else
        // {
        //     detected_target.region = detected_target.OUTSIDE;
        // }

        target_publisher.publish(detected_target);
    }
}
*/

String matchTarget(const clm_ros_wrapper::GazePointAndDirection & msg, clm_ros_wrapper::DetectedTarget & detected_target)
{
    string target = "";
    detected_target.certainty = msg.certainty;
    detected_target.match_distance = msg.role_confidence;

    tf::Vector3 gaze_point_wf, head_position_wf, hfv_wf;

    //checking if there is detection
    tf::vector3MsgToTF(msg.gaze_point, gaze_point_wf);
    tf::vector3MsgToTF(msg.head_position, head_position_wf);
    tf::vector3MsgToTF(msg.hfv, hfv_wf);

    if (gaze_point_wf.isZero() && head_position_wf.isZero() && hfv_wf.isZero())
    {
        //means no detection
        detected_target.distance = 0;
        detected_target.region = detected_target.NONE;
        target = "no detection";

        // cout << "no detection" << endl;
    }
    else
    {
        float _height = 1000;
        float _radian = 0.523599; //30 degree in radian

        double current_dist = 100000000.0;
        int estimated_region = detected_target.NONE;
        double shortest_dist = 100000000.0;
        int shortest_target = -1;
        bool found_a_match = false;

        if (msg.PARENT_ROLE == msg.role) {
            for (int _index = 0; _index < parent_virtual_targets.size(); _index++){
                if(is_point_inside_cone(head_position_wf, hfv_wf, _height, _radian, parent_virtual_targets.at(_index), current_dist)){
                    found_a_match = true;
                    if(current_dist < shortest_dist){
                        shortest_dist = current_dist;
                        shortest_target = _index;
                    }
                }
            }
        } else {
            for (int _index = 0; _index < child_virtual_targets.size(); _index++){
                if(is_point_inside_cone(head_position_wf, hfv_wf, _height, _radian, child_virtual_targets.at(_index), current_dist)){
                    found_a_match = true;
                    if(current_dist < shortest_dist){
                        shortest_dist = current_dist;
                        shortest_target = _index;
                    }
                }
            }
        }


        if(found_a_match){
            //cout << "shortest distance = " << shortest_dist << endl;
            if((shortest_target >= 0) && (shortest_target <= 11)){
                // std::cout << "..SCREEN " << shortest_target <<  std::endl;
                target = "screen";
                estimated_region = detected_target.SCREEN;
            }
            else if((shortest_target >= 12) && (shortest_target <= 23)){
                // std::cout << "...ROBOT " << shortest_target << std::endl;
                target = "robot";
                estimated_region = detected_target.ROBOT;
            }
            else {
                if (msg.PARENT_ROLE == msg.role) {
                    // std::cout << "...CHILD " << shortest_target << std::endl;
                    target = "child";
                    estimated_region = detected_target.CHILD;
                } else {
                    // std::cout << "...PARENT "<< shortest_target << std::endl;
                    target = "parent";
                    estimated_region = detected_target.PARENT;
                }
            }
        }
        else{
            // std::cout << "...OTHERS" << std::endl;
            estimated_region = detected_target.OUTSIDE;
            target = "other";
        }

        detected_target.region = estimated_region;
    }
    return target;
}

void assessment_callback(const clm_ros_wrapper::Assessment::ConstPtr& msg)
{
    if (msg->task != msg->TARGET) {
        return;
    }

    std::string parent_role = "";
    if (msg->PARENT_MOM == msg->parent_role) {
        parent_role = "mom/";
    } else if (msg->PARENT_DAD == msg->parent_role) {
        parent_role = "dad/";
    }

    std::string file_name = "/home/sar/face_analyzer_assessment/both_faces/" + parent_role;
    if (!exists(file_name)) {
        create_directories(file_name);
    }

    if (msg->state == msg->START) {
        assessment_child_total_num = 0;
        assessment_child_total_detected_num = 0;
        assessment_child_correct_num = 0;
        assessment_parent_total_num = 0;
        assessment_parent_total_detected_num = 0;
        assessment_parent_correct_num = 0;
        string child_file_name = "child_";
        string parent_file_name = "parent_";
        if (msg->task_content == msg->SCREEN) {
            child_file_name += "screen.txt";
            parent_file_name += "screen.txt";
            assessment_child_correct_answer = "screen";
            assessment_parent_correct_answer = "screen";
        } else if (msg->task_content == msg->ROBOT) {
            child_file_name += "robot.txt";
            parent_file_name += "robot.txt";
            assessment_child_correct_answer = "robot";
            assessment_parent_correct_answer = "robot";
        } else if (msg->task_content == msg->HUMAN) {
            child_file_name += "human.txt";
            parent_file_name += "human.txt";
            assessment_child_correct_answer = "parent";
            assessment_parent_correct_answer = "child";
        } else if (msg->task_content == msg->OTHER) {
            child_file_name += "other.txt";
            parent_file_name += "other.txt";
            assessment_child_correct_answer = "other";
            assessment_parent_correct_answer = "other";
        }

        assessment_child_file.open(file_name + child_file_name);
        assessment_parent_file.open(file_name + parent_file_name);
        is_assessing = true;
    } else if (msg->state == msg->END) {
        is_assessing = false;

        assessment_child_file << "accuracy" << endl;
        if (0 == assessment_child_total_num) {
            assessment_child_file << 0 << endl;
        } else {
            assessment_child_file << (double) assessment_child_correct_num / (double)assessment_child_total_num << endl;
        }
        assessment_child_file << "accuracy without no detection" << endl;
        if (0 == assessment_child_total_detected_num) {
            assessment_child_file << 0 << endl;
        } else {
            assessment_child_file << (double) assessment_child_correct_num / (double)assessment_child_total_detected_num << endl;
        }
        assessment_child_file.close();

        assessment_parent_file << "accuracy" << endl;
        if(0 == assessment_parent_total_num) {
            assessment_parent_file << 0 << endl;
        } else {
            assessment_parent_file << (double)assessment_parent_correct_num / (double)assessment_parent_total_num << endl;
        }
        assessment_parent_file << "accuracy without no detection" << endl;
        if (0 == assessment_parent_total_detected_num) {
            assessment_parent_file << 0 << endl;
        } else {
            assessment_parent_file << (double) assessment_parent_correct_num / (double)assessment_parent_total_detected_num << endl;
        }
        assessment_parent_file.close();
    }
}

void gazepoint_callback2(const clm_ros_wrapper::GazePointsAndDirections::ConstPtr& msg)
{
    clm_ros_wrapper::DetectedTargets detected_targets;
    detected_targets.targets.resize(msg->gazes.size());

    for(int loop = 0; loop < msg->gazes.size(); loop++)
    {
        clm_ros_wrapper::DetectedTarget detected_target;
        if (msg->gazes[loop].CHILD_ROLE == msg->gazes[loop].role) {
            detected_target.role = detected_target.CHILD_ROLE;
        } else if (msg->gazes[loop].PARENT_ROLE == msg->gazes[loop].role) {
            detected_target.role = detected_target.PARENT_ROLE;
        } else {
            detected_target.role = detected_target.OTHER_ROLE;
        }
        string target = matchTarget(msg->gazes[loop], detected_target);

        if (is_assessing) {
            if (detected_target.role == detected_target.CHILD_ROLE) {
                assessment_child_total_num++;
                assessment_child_file << target << endl;
                // cout << "child: " << target << endl;
                if (assessment_child_correct_answer == target) {
                    assessment_child_correct_num++;
                }
                if ("no detection" != target) {
                    assessment_child_total_detected_num++;
                }
            } else if (detected_target.role == detected_target.PARENT_ROLE) {
                assessment_parent_total_num++;
                assessment_parent_file << target << endl;
                // cout << "parent: " << target << endl;
                if (assessment_parent_correct_answer == target) {
                    assessment_parent_correct_num++;
                }
                if ("no detection" != target) {
                    assessment_parent_total_detected_num++;
                }
            }
        }

        detected_targets.targets[loop] = detected_target;

        // if (msg->gazes[loop].CHILD_ROLE == msg->gazes[loop].role) {
        //     target_publisher.publish(detected_target);
        // } else if (msg->gazes[loop].PARENT_ROLE == msg->gazes[loop].role) {
        // 	parent_target_publisher.publish(detected_target);
        // }
    }

    targets_publisher.publish(detected_targets);

    // double detection_certainty = (*msg).certainty;
    // double match_distance = (*msg).role_confidence;
    // tf::Vector3 gaze_point_wf, head_position_wf, hfv_wf;

    // //checking if there is detection
    // tf::vector3MsgToTF((*msg).gaze_point, gaze_point_wf);
    // tf::vector3MsgToTF((*msg).head_position, head_position_wf);
    // tf::vector3MsgToTF((*msg).hfv, hfv_wf);

    // if (gaze_point_wf.isZero() && head_position_wf.isZero() && hfv_wf.isZero())
    // {
    //     //means no detection
    //     clm_ros_wrapper::DetectedTarget target_no_detection;
    //     target_no_detection.certainty = detection_certainty;
    //     target_no_detection.match_distance = match_distance;
    //     target_no_detection.role = target_no_detection.CHILD_ROLE;

    //     target_no_detection.name = "NO DETECTION";
    //     target_no_detection.distance = 0;
    //     target_no_detection.region = target_no_detection.NONE;

    //     // cout << "no detection" << endl;
    //     target_publisher.publish(target_no_detection);
    // }
    // else
    // {
    //     clm_ros_wrapper::DetectedTarget detected_target;
    //     detected_target.certainty = detection_certainty;
    //     detected_target.match_distance = match_distance;
    //     detected_target.role = detected_target.CHILD_ROLE;
    //     float _height = 1000;
    //     float _radian = 0.523599; //30 degree in radian

    //     double current_dist = 100000000.0;
    //     int estimated_region = detected_target.NONE;
    //     double shortest_dist = 100000000.0;
    //     int shortest_target = -1;
    //     bool found_a_match = false;

    //     for (int _index = 0; _index < virtual_targets.size(); _index++){
    //         if(is_point_inside_cone(head_position_wf, hfv_wf, _height, _radian, virtual_targets.at(_index), current_dist)){
    //             found_a_match = true;
    //             if(current_dist < shortest_dist){
    //                 shortest_dist = current_dist;
    //                 shortest_target = _index;
    //             }
    //         }

    //     }

    //     if(found_a_match){
    //         //cout << "shortest distance = " << shortest_dist << endl;
    //         if((shortest_target >= 0) && (shortest_target <= 11)){
    //             std::cout << "..SCREEN" << std::endl;
    //             estimated_region = detected_target.SCREEN;
    //         }
    //         else if((shortest_target == 12) || (shortest_target == 13)){
    //             std::cout << "...ROBOT" << std::endl;
    //             estimated_region = detected_target.ROBOT;
    //         }
    //         else if((shortest_target == 14) || (shortest_target == 15)){
    //             std::cout << "...PARENT" << std::endl;
    //             estimated_region = detected_target.PARENT;
    //         }
    //     }
    //     else{
    //         std::cout << "...OTHERS" << std::endl;
    //         estimated_region = detected_target.OUTSIDE;
    //     }

    //     detected_target.region = estimated_region;
}

/*
void parent_gazepoint_callback2(const clm_ros_wrapper::GazePointAndDirection::ConstPtr& msg)
{
    double detection_certainty = (*msg).certainty;
    double match_distance = (*msg).role_confidence;
    tf::Vector3 gaze_point_wf, head_position_wf, hfv_wf;

    //checking if there is detection
    tf::vector3MsgToTF((*msg).gaze_point, gaze_point_wf);
    tf::vector3MsgToTF((*msg).head_position, head_position_wf);
    tf::vector3MsgToTF((*msg).hfv, hfv_wf);

    if (gaze_point_wf.isZero() && head_position_wf.isZero() && hfv_wf.isZero())
    {
        //means no detection
        clm_ros_wrapper::DetectedTarget target_no_detection;
        target_no_detection.certainty = detection_certainty;
        target_no_detection.role = target_no_detection.PARENT_ROLE;
        target_no_detection.match_distance = match_distance;
        target_no_detection.name = "NO DETECTION";
        target_no_detection.distance = 0;
        target_no_detection.region = target_no_detection.NONE;

        // cout << "parent  - no detection" << endl;
        parent_target_publisher.publish(target_no_detection);
    }
    else
    {
        clm_ros_wrapper::DetectedTarget detected_target;
        detected_target.certainty = detection_certainty;
        detected_target.match_distance = match_distance;
        detected_target.role = detected_target.PARENT_ROLE;
        float _height = 1000;
        float _radian = 0.523599; //30 degree in radian

        double current_dist = 100000000.0;
        int estimated_region = detected_target.NONE;
        double shortest_dist = 100000000.0;
        int shortest_target = -1;
        bool found_a_match = false;

        for (int _index = 0; _index < virtual_targets.size(); _index++){
            if(is_point_inside_cone(head_position_wf, hfv_wf, _height, _radian, virtual_targets.at(_index), current_dist)){
                found_a_match = true;
                if(current_dist < shortest_dist){
                    shortest_dist = current_dist;
                    shortest_target = _index;
                }
            }

        }

        if(found_a_match){
            //cout << "shortest distance = " << shortest_dist << endl;
            if((shortest_target >= 0) && (shortest_target <= 11)){
                // std::cout << "..parent..screen" << std::endl;
                estimated_region = detected_target.SCREEN;
            }
            else if((shortest_target == 12) || (shortest_target == 13)){
                // std::cout << "..parent..robot" << std::endl;
                estimated_region = detected_target.ROBOT;
            }
            else if((shortest_target == 16) || (shortest_target == 17)){
                // std::cout << "..parent..child" << std::endl;
                estimated_region = detected_target.CHILD;
            }
        }
        else{
            // std::cout << "..parent..others" << std::endl;
            estimated_region = detected_target.OUTSIDE;
        }

        detected_target.region = estimated_region;
        target_publisher.publish(detected_target);
    }
}
*/

void scene_callback(const clm_ros_wrapper::Scene::ConstPtr& msg)
{
    num_objects_on_screen = (*msg).screen.num_objects_on_screen;
    num_free_objects = (*msg).num_free_objects;

    // the objects on the screen
    for (int i =0; i < (*msg).screen.num_objects_on_screen ; i++)
    {
        //converting 2D coordinates of points on the screen to 3D points in the world frame
        screen_reference_points_wf[i] = tf::Vector3 ((*msg).screen.objects_on_screen[i].x_screen - screenWidth/2 + screenGap,
            cos(screenAngle) * (screenHeight - screenGap - (*msg).screen.objects_on_screen[i].y_screen),
            sin(screenAngle) * (screenHeight - screenGap - (*msg).screen.objects_on_screen[i].y_screen));

       screen_reference_points_names[i] =  std::string((*msg).screen.objects_on_screen[i].name);
    }

    screen_reference_points_names[(*msg).screen.num_objects_on_screen] =  "OUTSIDE";

    // the free objects
    for (int i =0; i < (*msg).num_free_objects; i++)
    {
        //free objects' positions are already given in world frame
        tf::vector3MsgToTF((*msg).free_objects[i].position, free_objects_positions[i]);
        free_objects_names[i] = std::string((*msg).free_objects[i].name);
    }

    free_objects_names[(*msg).num_free_objects] =  "OUTSIDE";
}


void system_callback(const sar_core::SystemState::ConstPtr& msg)
{
    int _state = (*msg).system_state;

    if(_state == (*msg).SYSTEM_DOWN){//7: SYSTEM_DOWN
        ros::shutdown();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "target_detector");


    ros::NodeHandle nh;

    nh.getParam("ns", _namespace);

    nh.getParam("screenAngleInDegrees", screenAngleInDegrees);
    nh.getParam("screenWidth", screenWidth);
    nh.getParam("screenHeight", screenHeight);
    nh.getParam("screenGap", screenGap);

    std::vector<float> transformation_wf2rf_param_ser;

    nh.getParam("transformation_wf2rf", transformation_wf2rf_param_ser);

    float robot_radius = robot_position_wf_z;
    free_object_radius = robot_radius; // TODO: need to change this parameter

    nh.getParam("robot_x_wf", robot_position_wf_x);
    nh.getParam("robot_y_wf", robot_position_wf_y);
    nh.getParam("robot_z_wf", robot_position_wf_z);

    screenAngle = screenAngleInDegrees * M_PI_2 / 90;

    targets_publisher = nh.advertise<clm_ros_wrapper::DetectedTargets>("/sar/perception/detect_targets", 1);
    // target_publisher = nh.advertise<clm_ros_wrapper::DetectedTarget>("/sar/perception/detect_target", 1);

    // parent_target_publisher = nh.advertise<clm_ros_wrapper::DetectedTarget>("/sar/perception/parent_detect_target", 1);

    ros::Subscriber system_sub = nh.subscribe("/sar/system/state", 1, &system_callback);

    ros::Subscriber scene = nh.subscribe("/sar/perception/scene", 1, &scene_callback);

    ros::Subscriber gazepoint_sub = nh.subscribe("/sar/perception/gaze_point_and_direction_wf", 1, &gazepoint_callback2);

    is_assessing = false;
    ros::Subscriber assessment_sub = nh.subscribe("/sar/perception/left_cam/clm_ros_wrapper_0/assessment", 1, &assessment_callback);

    // ros::Subscriber parent_gazepoint_sub = nh.subscribe("/sar/perception/parent_gaze_point_and_direction_wf", 1, &parent_gazepoint_callback2);

    // initialize virtual targets
    // tf::Vector3 virtual_screen_r1_c1 = tf::Vector3(64,190,251);
    // tf::Vector3 virtual_screen_r2_c1 = tf::Vector3(82,145,180);
    // tf::Vector3 virtual_screen_r3_c1 = tf::Vector3(100,100,110);

    // tf::Vector3 virtual_screen_r1_c2 = tf::Vector3(169,246,251);
    // tf::Vector3 virtual_screen_r2_c2 = tf::Vector3(187,201,180);
    // tf::Vector3 virtual_screen_r3_c2 = tf::Vector3(205,156,110);

    // tf::Vector3 virtual_screen_r1_c3 = tf::Vector3(275,303,251);
    // tf::Vector3 virtual_screen_r2_c3 = tf::Vector3(293,258,180);
    // tf::Vector3 virtual_screen_r3_c3 = tf::Vector3(311,213,110);

    // tf::Vector3 virtual_screen_r1_c4 = tf::Vector3(381,360,251);
    // tf::Vector3 virtual_screen_r2_c4 = tf::Vector3(399,315,180);
    // tf::Vector3 virtual_screen_r3_c4 = tf::Vector3(417,270,110);

    tf::Vector3 child_virtual_screen_r1_c1 = tf::Vector3(64,190,251);
    tf::Vector3 child_virtual_screen_r2_c1 = tf::Vector3(82,145,180);
    tf::Vector3 child_virtual_screen_r3_c1 = tf::Vector3(100,100,110);

    tf::Vector3 child_virtual_screen_r1_c2 = tf::Vector3(169,246,251);
    tf::Vector3 child_virtual_screen_r2_c2 = tf::Vector3(187,201,180);
    tf::Vector3 child_virtual_screen_r3_c2 = tf::Vector3(205,156,110);

    tf::Vector3 child_virtual_screen_r1_c3 = tf::Vector3(275,303,251);
    tf::Vector3 child_virtual_screen_r2_c3 = tf::Vector3(293,258,180);
    tf::Vector3 child_virtual_screen_r3_c3 = tf::Vector3(311,213,110);

    tf::Vector3 child_virtual_screen_r1_c4 = tf::Vector3(381,360,251);
    tf::Vector3 child_virtual_screen_r2_c4 = tf::Vector3(399,315,180);
    tf::Vector3 child_virtual_screen_r3_c4 = tf::Vector3(417,270,110);


    //tf::Vector3 virtual_screen_top_left = tf::Vector3(50,200,340);
    // tf::Vector3 virtual_screen_top_right = tf::Vector3(400,380,260);
    //tf::Vector3 virtual_screen_top_right = tf::Vector3(370,350,340);
    // tf::Vector3 virtual_screen_bottom_left = tf::Vector3(100,100,130);
    //tf::Vector3 virtual_screen_bottom_left = tf::Vector3(50,140,130);
    // tf::Vector3 virtual_screen_bottom_right = tf::Vector3(450,280,100);
    //tf::Vector3 virtual_screen_bottom_right = tf::Vector3(420,310,150);

    // tf::Vector3 child_virtual_robot_1 = tf::Vector3(630,250,50);
    // tf::Vector3 child_virtual_robot_2 = tf::Vector3(630,250,330);
    tf::Vector3 child_virtual_robot_1 = tf::Vector3(560,180,0);
    tf::Vector3 child_virtual_robot_2 = tf::Vector3(560,180,100);
    tf::Vector3 child_virtual_robot_3 = tf::Vector3(560,180,250);
    tf::Vector3 child_virtual_robot_4 = tf::Vector3(560,180,330);
    tf::Vector3 child_virtual_robot_5 = tf::Vector3(635,100,0);
    tf::Vector3 child_virtual_robot_6 = tf::Vector3(635,100,100);
    tf::Vector3 child_virtual_robot_7 = tf::Vector3(635,100,250);
    tf::Vector3 child_virtual_robot_8 = tf::Vector3(635,100,330);
    tf::Vector3 child_virtual_robot_9 = tf::Vector3(695,130,0);
    tf::Vector3 child_virtual_robot_10 = tf::Vector3(695,130,100);
    tf::Vector3 child_virtual_robot_11 = tf::Vector3(695,130,250);
    tf::Vector3 child_virtual_robot_12 = tf::Vector3(695,130,330);

    // tf::Vector3 child_virtual_parent_1 = tf::Vector3(730,-100,400);
    // tf::Vector3 child_virtual_parent_2 = tf::Vector3(730,0,400);
    // tf::Vector3 child_virtual_parent_3 = tf::Vector3(730,-100,150);
    // tf::Vector3 child_virtual_parent_4 = tf::Vector3(730,0,150);
    // tf::Vector3 child_virtual_parent_5 = tf::Vector3(730,-100,50);
    // tf::Vector3 child_virtual_parent_6 = tf::Vector3(730,0,550);

    // virtual_targets.push_back(virtual_screen_center);
    // virtual_targets.push_back(virtual_screen_top_left);
    // virtual_targets.push_back(virtual_screen_top_right);
    // virtual_targets.push_back(virtual_screen_bottom_left);
    // virtual_targets.push_back(virtual_screen_bottom_right);
    child_virtual_targets.push_back(child_virtual_screen_r1_c1);
    child_virtual_targets.push_back(child_virtual_screen_r2_c1);
    child_virtual_targets.push_back(child_virtual_screen_r3_c1);
    child_virtual_targets.push_back(child_virtual_screen_r1_c2);
    child_virtual_targets.push_back(child_virtual_screen_r2_c2);
    child_virtual_targets.push_back(child_virtual_screen_r3_c2);
    child_virtual_targets.push_back(child_virtual_screen_r1_c3);
    child_virtual_targets.push_back(child_virtual_screen_r2_c3);
    child_virtual_targets.push_back(child_virtual_screen_r3_c3);
    child_virtual_targets.push_back(child_virtual_screen_r1_c4);
    child_virtual_targets.push_back(child_virtual_screen_r2_c4);
    child_virtual_targets.push_back(child_virtual_screen_r3_c4);

    child_virtual_targets.push_back(child_virtual_robot_1);
    child_virtual_targets.push_back(child_virtual_robot_2);
    child_virtual_targets.push_back(child_virtual_robot_3);
    child_virtual_targets.push_back(child_virtual_robot_4);
    child_virtual_targets.push_back(child_virtual_robot_5);
    child_virtual_targets.push_back(child_virtual_robot_6);
    child_virtual_targets.push_back(child_virtual_robot_7);
    child_virtual_targets.push_back(child_virtual_robot_8);
    child_virtual_targets.push_back(child_virtual_robot_9);
    child_virtual_targets.push_back(child_virtual_robot_10);
    child_virtual_targets.push_back(child_virtual_robot_11);
    child_virtual_targets.push_back(child_virtual_robot_12);

    // child_virtual_targets.push_back(child_virtual_parent_1);
    // child_virtual_targets.push_back(child_virtual_parent_2);
    // child_virtual_targets.push_back(child_virtual_parent_3);
    // child_virtual_targets.push_back(child_virtual_parent_4);
    // child_virtual_targets.push_back(child_virtual_parent_5);
    // child_virtual_targets.push_back(child_virtual_parent_6);

    // push in parent point
    for (int x = 650; x <= 1150; x += 100) {
        for (int y = -200; y >= -700; y -= 100) {
            for (int z = 100; z <= 800; z += 100) {
                child_virtual_targets.push_back(tf::Vector3(x, y, z));
            }
        }
    }

    tf::Vector3 parent_virtual_screen_r1_c1 = tf::Vector3(33,380,374);
    tf::Vector3 parent_virtual_screen_r2_c1 = tf::Vector3(66,295,242);
    tf::Vector3 parent_virtual_screen_r3_c1 = tf::Vector3(100,210,110);

    tf::Vector3 parent_virtual_screen_r1_c2 = tf::Vector3(138,336,374);
    tf::Vector3 parent_virtual_screen_r2_c2 = tf::Vector3(171,251,242);
    tf::Vector3 parent_virtual_screen_r3_c2 = tf::Vector3(205,166,110);

    tf::Vector3 parent_virtual_screen_r1_c3 = tf::Vector3(244,393,374);
    tf::Vector3 parent_virtual_screen_r2_c3 = tf::Vector3(277,308,242);
    tf::Vector3 parent_virtual_screen_r3_c3 = tf::Vector3(311,223,110);

    tf::Vector3 parent_virtual_screen_r1_c4 = tf::Vector3(350,450,374);
    tf::Vector3 parent_virtual_screen_r2_c4 = tf::Vector3(383,365,242);
    tf::Vector3 parent_virtual_screen_r3_c4 = tf::Vector3(417,280,110);

    // tf::Vector3 parent_virtual_robot_top = tf::Vector3(700,170,50);
    // tf::Vector3 parent_virtual_robot_bottom = tf::Vector3(700,170,330);
    tf::Vector3 parent_virtual_robot_1 = tf::Vector3(560,170,0);
    tf::Vector3 parent_virtual_robot_2 = tf::Vector3(560,170,140);
    tf::Vector3 parent_virtual_robot_3 = tf::Vector3(560,260,280);
    tf::Vector3 parent_virtual_robot_4 = tf::Vector3(560,260,450);
    tf::Vector3 parent_virtual_robot_5 = tf::Vector3(680,110,0);
    tf::Vector3 parent_virtual_robot_6 = tf::Vector3(680,110,140);
    tf::Vector3 parent_virtual_robot_7 = tf::Vector3(680,110,280);
    tf::Vector3 parent_virtual_robot_8 = tf::Vector3(680,110,450);
    tf::Vector3 parent_virtual_robot_9 = tf::Vector3(700,190,0);
    tf::Vector3 parent_virtual_robot_10 = tf::Vector3(700,190,140);
    tf::Vector3 parent_virtual_robot_11 = tf::Vector3(700,190,340);
    tf::Vector3 parent_virtual_robot_12 = tf::Vector3(700,190,450);

    // tf::Vector3 parent_virtual_child_1 = tf::Vector3(305,-60,430);
    // tf::Vector3 parent_virtual_child_2 = tf::Vector3(305,-60,150);
    // tf::Vector3 parent_virtual_child_3 = tf::Vector3(305,-60,0);
    // tf::Vector3 parent_virtual_child_4 = tf::Vector3(305,0,430);
    // tf::Vector3 parent_virtual_child_5 = tf::Vector3(305,0,150);
    // tf::Vector3 parent_virtual_child_6 = tf::Vector3(305,0,0);

    parent_virtual_targets.push_back(parent_virtual_screen_r1_c1);
    parent_virtual_targets.push_back(parent_virtual_screen_r2_c1);
    parent_virtual_targets.push_back(parent_virtual_screen_r3_c1);
    parent_virtual_targets.push_back(parent_virtual_screen_r1_c2);
    parent_virtual_targets.push_back(parent_virtual_screen_r2_c2);
    parent_virtual_targets.push_back(parent_virtual_screen_r3_c2);
    parent_virtual_targets.push_back(parent_virtual_screen_r1_c3);
    parent_virtual_targets.push_back(parent_virtual_screen_r2_c3);
    parent_virtual_targets.push_back(parent_virtual_screen_r3_c3);
    parent_virtual_targets.push_back(parent_virtual_screen_r1_c4);
    parent_virtual_targets.push_back(parent_virtual_screen_r2_c4);
    parent_virtual_targets.push_back(parent_virtual_screen_r3_c4);

    parent_virtual_targets.push_back(parent_virtual_robot_1);
    parent_virtual_targets.push_back(parent_virtual_robot_2);
    parent_virtual_targets.push_back(parent_virtual_robot_3);
    parent_virtual_targets.push_back(parent_virtual_robot_4);
    parent_virtual_targets.push_back(parent_virtual_robot_5);
    parent_virtual_targets.push_back(parent_virtual_robot_6);
    parent_virtual_targets.push_back(parent_virtual_robot_7);
    parent_virtual_targets.push_back(parent_virtual_robot_8);
    parent_virtual_targets.push_back(parent_virtual_robot_9);
    parent_virtual_targets.push_back(parent_virtual_robot_10);
    parent_virtual_targets.push_back(parent_virtual_robot_11);
    parent_virtual_targets.push_back(parent_virtual_robot_12);

    // parent_virtual_targets.push_back(parent_virtual_child_1);
    // parent_virtual_targets.push_back(parent_virtual_child_2);
    // parent_virtual_targets.push_back(parent_virtual_child_3);
    // parent_virtual_targets.push_back(parent_virtual_child_4);
    // parent_virtual_targets.push_back(parent_virtual_child_5);
    // parent_virtual_targets.push_back(parent_virtual_child_6);

    // push in child point
    for (int x = 0; x <= 100; x += 100) {
        for (int y = -200; y >= -700; y -= 100) {
            for (int z = 100; z <= 400; z += 100) {
                parent_virtual_targets.push_back(tf::Vector3(x, y, z));
            }
        }
    }

    for (int x = 200; x <= 300; x += 100) {
        for (int y = -300; y >= -700; y -= 100) {
            for (int z = 100; z <= 400; z += 100) {
                parent_virtual_targets.push_back(tf::Vector3(x, y, z));
            }
        }
    }

    for (int x = 400; x <= 500; x += 100) {
        for (int y = -400; y >= -700; y -= 100) {
            for (int z = 100; z <= 400; z += 100) {
                parent_virtual_targets.push_back(tf::Vector3(x, y, z));
            }
        }
    }


    for (int x = 500; x <= 600; x += 100) {
        for (int y = -400; y >= -700; y -= 100) {
            for (int z = 100; z <= 400; z += 100) {
                parent_virtual_targets.push_back(tf::Vector3(x, y, z));
            }
        }
    }


    ros::spin();
}
