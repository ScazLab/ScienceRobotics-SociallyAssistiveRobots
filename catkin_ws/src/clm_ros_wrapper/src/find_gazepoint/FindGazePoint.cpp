/*
This ros node subscribes to the topics "/clm_ros_wrapper/head_position" (for headposition in camera frame)
and "/clm_ros_wrapper/head_vector" (for head fixation vector in camera frame) and computes the intersection
point of the head direction and the hardcoded screen. It then publishes this geometry_msgs::Vector3 with
publisher topic "/clm_ros_wrapper/gaze_point".
Yunus
*/
#include "CLM_core.h"

#include <fstream>
#include <sstream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <Face_utils.h>
#include <FaceAnalyser.h>
#include <GazeEstimation.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <clm_ros_wrapper/ClmHeads.h>
#include <clm_ros_wrapper/ClmEyeGaze.h>
#include <clm_ros_wrapper/GazeDirection.h>
#include <clm_ros_wrapper/GazeDirections.h>
#include <clm_ros_wrapper/ClmFacialActionUnit.h>
#include <clm_ros_wrapper/GazePointAndDirection.h>
#include <clm_ros_wrapper/GazePointsAndDirections.h>
#include <clm_ros_wrapper/VectorWithCertainty.h>
#include <clm_ros_wrapper/VectorsWithCertainty.h>
#include <clm_ros_wrapper/ClmHeadVectors.h>
#include <clm_ros_wrapper/VectorsWithCertaintyWithGazePointsAndDirections.h>
#include <sar_core/SystemState.h>

#include <filesystem.hpp>
#include <filesystem/fstream.hpp>

#include <math.h>

#include <tf/transform_datatypes.h>

#include <geometry_msgs/Vector3.h>

int screenAngleInDegrees;
float screenWidth;
float screenHeight;
float screenAngle;
string _namespace;

using namespace std;
using namespace cv;

using namespace boost::filesystem;
using std::vector;

//ros::Publisher gaze_point_and_direction_pub;
//ros::Publisher head_position_rf_pub;
ros::Publisher gaze_direction_wf_pub;
ros::Publisher headposition_and_gaze_pub;

//tf::Vector3 headposition_cf;

tf::Vector3 left_gaze_direction_cf_tf;
tf::Vector3 right_gaze_direction_cf_tf;

cv::Matx<float, 4, 4> transformation_cf2intermediate_frame;
cv::Matx<float, 4, 4> transformation_intermediate_frame2wf;
cv::Matx<float, 4, 4> transformation_wf2rf;

std::vector<double> transformation_matrix_cf2intermediate_frame_array_parameter_server;
std::vector<double> transformation_matrix_intermediate_frame2wf_array_parameter_server;
std::vector<double> transformation_matrix_wf2rf_array_parameter_server;
std::vector<float> detection_certainties;
std::vector<tf::Vector3> headpositions_cf;
//float detection_certainty;

float offset_hfv_wf_z;
float offset_head_position_cf_z;

tf::Vector3 vector3_cv2tf(cv::Matx<float, 4, 1> vector_cv)
{
    tf::Vector3 vector_tf;
    vector_tf.setX(vector_cv(0,0));
    vector_tf.setY(vector_cv(1,0));
    vector_tf.setZ(vector_cv(2,0));
    return vector_tf;
}

cv::Matx<float, 4, 1> vector3_tf2cv(tf::Vector3 vector_tf, bool isPoint)
{
    if (isPoint)
    {
        return cv::Matx<float, 4, 1>(vector_tf.getX(), vector_tf.getY(), vector_tf.getZ(), 1);
    }
    else
    {
        return cv::Matx<float, 4, 1>(vector_tf.getX(), vector_tf.getY(), vector_tf.getZ(), 0);
    }
}

void vector_callback(const clm_ros_wrapper::ClmHeadVectors::ConstPtr& msg)
{
    //clm_ros_wrapper::GazePointsAndDirections gazes_pd_msg;
    //clm_ros_wrapper::VectorsWithCertainty head_positions_rf_with_certainty;
    clm_ros_wrapper::VectorsWithCertaintyWithGazePointsAndDirections publish_msg;
    publish_msg.gazes.resize(msg->head_vectors.size());
    publish_msg.vectors_rf.resize((*msg).head_vectors.size());
    publish_msg.head_position_wf.resize((*msg).head_vectors.size());

    for(int loop = 0; loop < msg->head_vectors.size(); loop++)
    {
        tf::Vector3 hfv_cf;
        geometry_msgs::Vector3 head_vector_msg;
        head_vector_msg = msg->head_vectors[loop];
        tf::vector3MsgToTF(head_vector_msg, hfv_cf);
        publish_msg.gazes[loop].role = msg->roles[loop];
        publish_msg.gazes[loop].role_confidence = msg->role_confidences[loop];

        //checking if there is a detection
        if (headpositions_cf.size() == 0 || headpositions_cf[loop].isZero() || hfv_cf.isZero())
        {
            // no face detection publish the message with 3 zero vectors
            tf::Vector3 zero_vector = tf::Vector3(0, 0, 0);
            tf::vector3TFToMsg(zero_vector, publish_msg.gazes[loop].gaze_point);
            tf::vector3TFToMsg(zero_vector, publish_msg.gazes[loop].head_position);
            tf::vector3TFToMsg(zero_vector, publish_msg.head_position_wf[loop]);
            tf::vector3TFToMsg(zero_vector, publish_msg.gazes[loop].hfv);
            if (detection_certainties.size() > loop)
            {
                publish_msg.gazes[loop].certainty = detection_certainties[loop];
            }
            else
            {
                publish_msg.gazes[loop].certainty = 0.0;
            }
            //gaze_point_and_direction_pub.publish(gaze_pd_msg);

            //for head position in the robot frame
            geometry_msgs::Vector3 zero_msg;
            tf::vector3TFToMsg(zero_vector, zero_msg);
            publish_msg.vectors_rf[loop].position = zero_msg;

            if (detection_certainties.size() > loop)
            {
                publish_msg.vectors_rf[loop].certainty = detection_certainties[loop];
            }
            else
            {
             publish_msg.vectors_rf[loop].certainty = 0.0;
            }
            //head_position_rf_pub.publish(head_position_rf_with_certainty);
        }

        else
        {
            cv::Matx<float,4,4> transformation_matrix_cf2wf = transformation_cf2intermediate_frame * transformation_intermediate_frame2wf;
            tf::Vector3 hfv_wf = vector3_cv2tf(transformation_matrix_cf2wf * (vector3_tf2cv(hfv_cf, 0)));

            // correcting the Z element of the head fixation vector from CLM
            hfv_wf.setZ(hfv_wf.getZ() + offset_hfv_wf_z);

            cv::Matx<float, 4, 1> headposition_wf_cv = transformation_matrix_cf2wf * (vector3_tf2cv(headpositions_cf[loop], 1));
            tf::Vector3 headposition_wf = vector3_cv2tf(headposition_wf_cv);

            //head position robot frame
            tf::Vector3 head_position_rf = vector3_cv2tf(transformation_wf2rf*vector3_tf2cv(headposition_wf, 1));

            //publishing the head position in the robot frame
            clm_ros_wrapper::VectorWithCertainty head_position_rf_with_certainty;

            geometry_msgs::Vector3 head_position_rf_msg;
            tf::vector3TFToMsg(head_position_rf, head_position_rf_msg);

            publish_msg.vectors_rf[loop].position = head_position_rf_msg;
            publish_msg.vectors_rf[loop].certainty = detection_certainties[loop];
            //head_position_rf_pub.publish(head_position_rf_with_certainty);


            //Below is the calculation of the gazepoint
            tf::Vector3 randompoint_on_gazedirection_wf = headposition_wf + 500 * hfv_wf;

            //hacky way
            // if((hfv_wf.getX() <= 0.3) && (hfv_wf.getX() >= -0.2) && (hfv_wf.getZ() >= -0.25) && ( hfv_wf.getZ()<=0)){
            //     cout << "SCREEN" << endl;
            // }
            // else if((hfv_wf.getX() <= 0.5) && (hfv_wf.getX() >= 0.4) && (hfv_wf.getZ() >= -0.25) && ( hfv_wf.getZ()<=0)){
            //     cout << "ROBOT" << endl;
            // }
            // else {
            //     cout << "out side" << endl;
            // }


            //storing the locations of the lower corners of screen and the camera
            //in world frame to establish the space where it sits
            tf::Vector3 lower_left_corner_of_screen_wf = tf::Vector3(145, 25, 0);
            tf::Vector3 lower_right_corner_of_screen_wf = tf::Vector3(452, 188, 0);
            tf::Vector3 upper_mid__point_of_screen_wf = tf::Vector3(50,25,50);

            // using the Line-Plane intersection formula on Wolfram link: http://mathworld.wolfram.com/Line-PlaneIntersection.html
            // ALL CALCULATIONS ARE MADE IN WORLD FRAME
            // Explanation: To construct the line to intersect, I take two points in the gaze direction, the camera location and another point that is equal to camera point
            // plus a constant times the head fixation vector -- this extra point is named randompoint_on_gazedirection_wf.
            // with the notation from the link x4 = headposition_wf and x5 = randompoint_on_gazedirection_wf
            cv::Matx<float, 4,4> matrix1 = cv::Matx<float, 4, 4>(1, 1, 1, 1,
                upper_mid__point_of_screen_wf.getX(), lower_right_corner_of_screen_wf.getX(), lower_left_corner_of_screen_wf.getX(), headposition_wf.getX(),
                upper_mid__point_of_screen_wf.getY(), lower_right_corner_of_screen_wf.getY(), lower_left_corner_of_screen_wf.getY(), headposition_wf.getY(),
                upper_mid__point_of_screen_wf.getZ(), lower_right_corner_of_screen_wf.getZ(), lower_left_corner_of_screen_wf.getZ(), headposition_wf.getZ());

            cv::Matx<float, 4,4> matrix2 = cv::Matx<float, 4, 4>(1, 1, 1, 0,
                upper_mid__point_of_screen_wf.getX(), lower_right_corner_of_screen_wf.getX(), lower_left_corner_of_screen_wf.getX(), randompoint_on_gazedirection_wf.getX() - headposition_wf.getX(),
                upper_mid__point_of_screen_wf.getY(), lower_right_corner_of_screen_wf.getY(), lower_left_corner_of_screen_wf.getY(), randompoint_on_gazedirection_wf.getY() - headposition_wf.getY(),
                upper_mid__point_of_screen_wf.getZ(), lower_right_corner_of_screen_wf.getZ(), lower_left_corner_of_screen_wf.getZ(), randompoint_on_gazedirection_wf.getZ() - headposition_wf.getZ());

            // following the formula, I calculate t, which is determinant ratio -- check the link
            double determinant_ratio = (-1) * cv::determinant(matrix1) / cv::determinant(matrix2);

            // finally I plug in the determinant ratio (t) to get the intersection point
            tf::Vector3 gazepoint_on_screen_wf = headposition_wf + determinant_ratio * (randompoint_on_gazedirection_wf - headposition_wf);
            tf::vector3TFToMsg(gazepoint_on_screen_wf, publish_msg.gazes[loop].gaze_point);
            tf::vector3TFToMsg(headposition_wf, publish_msg.gazes[loop].head_position);
            tf::vector3TFToMsg(headposition_wf, publish_msg.head_position_wf[loop]);
            tf::vector3TFToMsg(hfv_wf, publish_msg.gazes[loop].hfv);
            publish_msg.gazes[loop].certainty = detection_certainties[loop];
            tf::Vector3 zero_vector = tf::Vector3(0,0,0);
            headpositions_cf[loop] = zero_vector;
            hfv_cf = zero_vector;
        }
    }

    //gaze_point_and_direction_pub.publish(gazes_pd_msg);
    //head_position_rf_pub.publish(head_positions_rf_with_certainty);
    headposition_and_gaze_pub.publish(publish_msg);
}
/**
void vector_callback(const geometry_msgs::Vector3::ConstPtr& msg)
{
    tf::Vector3 hfv_cf;

    tf::vector3MsgToTF(*msg, hfv_cf);

    //Message to publish pd -> point and direction
    clm_ros_wrapper::GazePointAndDirection gaze_pd_msg;

    //checking if there is a detection
    if (headposition_cf.isZero() || hfv_cf.isZero())
    {
        // no face detection publish the message with 3 zero vectors
        tf::Vector3 zero_vector = tf::Vector3(0, 0, 0);
        tf::vector3TFToMsg(zero_vector, gaze_pd_msg.gaze_point);
        tf::vector3TFToMsg(zero_vector, gaze_pd_msg.head_position);
        tf::vector3TFToMsg(zero_vector, gaze_pd_msg.hfv);
        gaze_pd_msg.certainty = detection_certainty;
        gaze_point_and_direction_pub.publish(gaze_pd_msg);

        //for head position in the robot frame
        geometry_msgs::Vector3 zero_msg;
        tf::vector3TFToMsg(zero_vector, zero_msg);

        clm_ros_wrapper::VectorWithCertainty head_position_rf_with_certainty;
        head_position_rf_with_certainty.position = zero_msg;
        head_position_rf_with_certainty.certainty = detection_certainty;
        head_position_rf_pub.publish(head_position_rf_with_certainty);
    }

    else
    {

        // rotation matrix from camera frame to world frame

        // rotation_matrix_cf2wf.setValue(-1, 0, 0, 0, 0, -1, 0, -1, 0);

        // tf::Matrix3x3 matrix_cf2wf_rotate_axis;

        // // in the new setting, the screen is slightly rotated, so
        // // I use this new matrix to do an extra rotation of the axis cf2wf
        // rotation_matrix_cf2wf_rotate_axis.setValue(0.897904, 0.0145582, -0.439951,
        //     -0.067152, 0.992285,  -0.104216,
        //     0.43504, 0.12312, 0.891954);

        //applying the extra rotation
        // matrix_cf2wf = matrix_cf2wf_rotate_axis * matrix_cf2wf;

        // translation vector from cf to wf
        //tf::Vector3 vector_cf2wf; //tf::Vector3((-1) * screenWidth/3, sin(screenAngle) * screenHeight, cos(screenAngle) * screenHeight);

        // transformation from the camera frame to the world frame
        // tf::Transform transfrom_cf2wf = tf::Transform(rotation_matrix_cf2wf, translation_vector_cf2wf);

        cv::Matx<float,4,4> transformation_matrix_cf2wf = transformation_cf2intermediate_frame * transformation_intermediate_frame2wf;
        //cout << transformation_matrix_cf2wf << endl;
        tf::Vector3 hfv_wf = vector3_cv2tf(transformation_matrix_cf2wf * (vector3_tf2cv(hfv_cf, 0)));
        //cout << vector3_tf2cv(hfv_cf, 0) << endl;

        // correcting the Z element of the head fixation vector from CLM
        hfv_wf.setZ(hfv_wf.getZ() + offset_hfv_wf_z);

        // testing
        //headposition_cf = tf::Vector3(-82, 350, 260);

        // storing the head position in the camera frame
        // /headposition_cf = tf::Vector3(0,0,500);

        // adding the box size offset_head_position_cf_z
        //headposition_cf = headposition_cf + tf::Vector3(0,0,offset_head_position_cf_z);

        cv::Matx<float, 4, 1> headposition_wf_cv = transformation_matrix_cf2wf * (vector3_tf2cv(headposition_cf, 1));
        tf::Vector3 headposition_wf = vector3_cv2tf(headposition_wf_cv);
        // cout << "hp wf X = " << headposition_wf.getX() << endl;
        // cout << "hp wf Y = " << headposition_wf.getY() << endl;
        // cout << "hp wf Z = " << headposition_wf.getZ() << endl;

        //head position robot frame
        tf::Vector3 head_position_rf = vector3_cv2tf(transformation_wf2rf*vector3_tf2cv(headposition_wf, 1));
        // cout << "hp rf X = " << head_position_rf.getX() << endl;
        // cout << "hp rf Y = " << head_position_rf.getY() << endl;
        // cout << "hp rf Z = " << head_position_rf.getZ() << endl;

        //publishing the head position in the robot frame
        clm_ros_wrapper::VectorWithCertainty head_position_rf_with_certainty;

        geometry_msgs::Vector3 head_position_rf_msg;
        tf::vector3TFToMsg(head_position_rf, head_position_rf_msg);

        head_position_rf_with_certainty.position = head_position_rf_msg;
        head_position_rf_with_certainty.certainty = detection_certainty;

        head_position_rf_pub.publish(head_position_rf_with_certainty);


        //Below is the calculation of the gazepoint
        tf::Vector3 randompoint_on_gazedirection_wf = headposition_wf + 500 * hfv_wf;
        // cout << "hfv wf X = " << hfv_wf.getX() << endl;
        // cout << "hfv wf Y = " << hfv_wf.getY() << endl;
        // cout << "hfv wf Z = " << hfv_wf.getZ() << endl;

        //hacky way
        // if((hfv_wf.getX() <= 0.3) && (hfv_wf.getX() >= -0.2) && (hfv_wf.getZ() >= -0.25) && ( hfv_wf.getZ()<=0)){
        //     cout << "SCREEN" << endl;
        // }
        // else if((hfv_wf.getX() <= 0.5) && (hfv_wf.getX() >= 0.4) && (hfv_wf.getZ() >= -0.25) && ( hfv_wf.getZ()<=0)){
        //     cout << "ROBOT" << endl;
        // }
        // else {
        //     cout << "out side" << endl;
        // }


        //storing the locations of the lower corners of screen and the camera
        //in world frame to establish the space where it sits
        tf::Vector3 lower_left_corner_of_screen_wf = tf::Vector3(145, 25, 0);
        tf::Vector3 lower_right_corner_of_screen_wf = tf::Vector3(452, 188, 0);
        tf::Vector3 upper_mid__point_of_screen_wf = tf::Vector3(50,25,50);

        // using the Line-Plane intersection formula on Wolfram link: http://mathworld.wolfram.com/Line-PlaneIntersection.html
        // ALL CALCULATIONS ARE MADE IN WORLD FRAME
        // Explanation: To construct the line to intersect, I take two points in the gaze direction, the camera location and another point that is equal to camera point
        // plus a constant times the head fixation vector -- this extra point is named randompoint_on_gazedirection_wf.
        // with the notation from the link x4 = headposition_wf and x5 = randompoint_on_gazedirection_wf
        cv::Matx<float, 4,4> matrix1 = cv::Matx<float, 4, 4>(1, 1, 1, 1,
            upper_mid__point_of_screen_wf.getX(), lower_right_corner_of_screen_wf.getX(), lower_left_corner_of_screen_wf.getX(), headposition_wf.getX(),
            upper_mid__point_of_screen_wf.getY(), lower_right_corner_of_screen_wf.getY(), lower_left_corner_of_screen_wf.getY(), headposition_wf.getY(),
            upper_mid__point_of_screen_wf.getZ(), lower_right_corner_of_screen_wf.getZ(), lower_left_corner_of_screen_wf.getZ(), headposition_wf.getZ());

        cv::Matx<float, 4,4> matrix2 = cv::Matx<float, 4, 4>(1, 1, 1, 0,
            upper_mid__point_of_screen_wf.getX(), lower_right_corner_of_screen_wf.getX(), lower_left_corner_of_screen_wf.getX(), randompoint_on_gazedirection_wf.getX() - headposition_wf.getX(),
            upper_mid__point_of_screen_wf.getY(), lower_right_corner_of_screen_wf.getY(), lower_left_corner_of_screen_wf.getY(), randompoint_on_gazedirection_wf.getY() - headposition_wf.getY(),
            upper_mid__point_of_screen_wf.getZ(), lower_right_corner_of_screen_wf.getZ(), lower_left_corner_of_screen_wf.getZ(), randompoint_on_gazedirection_wf.getZ() - headposition_wf.getZ());

        // following the formula, I calculate t, which is determinant ratio -- check the link
        double determinant_ratio = (-1) * cv::determinant(matrix1) / cv::determinant(matrix2);

        // finally I plug in the determinant ratio (t) to get the intersection point
        tf::Vector3 gazepoint_on_screen_wf = headposition_wf + determinant_ratio * (randompoint_on_gazedirection_wf - headposition_wf);
        // cout << "gaze x = " << gazepoint_on_screen_wf.getX() << endl;
        // cout << "gaze y = " << gazepoint_on_screen_wf.getY() << endl;
        // cout << "gaze z = " << gazepoint_on_screen_wf.getZ() << endl;


        tf::vector3TFToMsg(gazepoint_on_screen_wf, gaze_pd_msg.gaze_point);
        tf::vector3TFToMsg(headposition_wf, gaze_pd_msg.head_position);
        tf::vector3TFToMsg(hfv_wf, gaze_pd_msg.hfv);
        gaze_pd_msg.certainty = detection_certainty;
        gaze_point_and_direction_pub.publish(gaze_pd_msg);

        tf::Vector3 zero_vector = tf::Vector3(0,0,0);
        headposition_cf = zero_vector;
        hfv_cf = zero_vector;
    }
    //cout << "detection_certainty" << detection_certainty << endl;
}**/

void headposition_callback(const clm_ros_wrapper::VectorsWithCertainty::ConstPtr& msg)
{
    detection_certainties.resize(msg->vectors.size());
    headpositions_cf.resize(msg->vectors.size());
    for(int loop = 0; loop < msg->vectors.size(); loop++)
    {
        tf::vector3MsgToTF(msg->vectors[loop].position, headpositions_cf[loop]);
        detection_certainties[loop] = msg->vectors[loop].certainty;
    }
}

/**void headposition_callback(const clm_ros_wrapper::VectorWithCertainty::ConstPtr& msg)
{
    tf::vector3MsgToTF((*msg).position, headposition_cf);
    detection_certainty = (*msg).certainty;
}**/

void gaze_direction_callback(const clm_ros_wrapper::GazeDirections::ConstPtr& msg)
{
    clm_ros_wrapper::GazeDirections ros_gaze_directions_msg;
    ros_gaze_directions_msg.directions.resize(msg->directions.size());
    for(int loop = 0; loop < msg->directions.size(); loop++)
    {
        // TODO?: detection certainty. does not seem like clm provides this info
        clm_ros_wrapper::GazeDirection in_ros_gaze_direction_msg;
        in_ros_gaze_direction_msg = msg->directions[loop];
        tf::vector3MsgToTF(in_ros_gaze_direction_msg.left_gaze_diection, left_gaze_direction_cf_tf);
        tf::vector3MsgToTF(in_ros_gaze_direction_msg.right_gaze_diection, right_gaze_direction_cf_tf);

        // don't use the hf, head frame, for now

        // cf -> wf
        cv::Matx<float,4,4> transformation_matrix_cf2wf = transformation_cf2intermediate_frame * transformation_intermediate_frame2wf;
        tf::Vector3 left_gaze_direction_wf = vector3_cv2tf(transformation_matrix_cf2wf * (vector3_tf2cv(left_gaze_direction_cf_tf, 0)));
        tf::Vector3 right_gaze_direction_wf = vector3_cv2tf(transformation_matrix_cf2wf * (vector3_tf2cv(right_gaze_direction_cf_tf, 0)));

        // publish wf
        ros_gaze_directions_msg.directions[loop].left_gaze_diection.x = left_gaze_direction_wf.getX();
        ros_gaze_directions_msg.directions[loop].left_gaze_diection.y = left_gaze_direction_wf.getY();
        ros_gaze_directions_msg.directions[loop].left_gaze_diection.z = left_gaze_direction_wf.getZ();
        ros_gaze_directions_msg.directions[loop].right_gaze_diection.x = right_gaze_direction_wf.getX();
        ros_gaze_directions_msg.directions[loop].right_gaze_diection.y = right_gaze_direction_wf.getY();
        ros_gaze_directions_msg.directions[loop].right_gaze_diection.z = right_gaze_direction_wf.getZ();
    }
    gaze_direction_wf_pub.publish(ros_gaze_directions_msg);
}
/**void gaze_direction_callback(const clm_ros_wrapper::GazeDirection::ConstPtr& msg)
{
    // TODO?: detection certainty. does not seem like clm provides this info
    tf::vector3MsgToTF((*msg).left_gaze_diection, left_gaze_direction_cf_tf);
    tf::vector3MsgToTF((*msg).right_gaze_diection, right_gaze_direction_cf_tf);

    // don't use the hf, head frame, for now

    // cf -> wf
    cv::Matx<float,4,4> transformation_matrix_cf2wf = transformation_cf2intermediate_frame * transformation_intermediate_frame2wf;
    tf::Vector3 left_gaze_direction_wf = vector3_cv2tf(transformation_matrix_cf2wf * (vector3_tf2cv(left_gaze_direction_cf_tf, 0)));
    tf::Vector3 right_gaze_direction_wf = vector3_cv2tf(transformation_matrix_cf2wf * (vector3_tf2cv(right_gaze_direction_cf_tf, 0)));

    // publish wf
    clm_ros_wrapper::GazeDirection ros_gaze_direction_msg;
    geometry_msgs::Vector3 lefe_gaze_direction_wf_msg;
    lefe_gaze_direction_wf_msg.x = left_gaze_direction_wf.getX();
    lefe_gaze_direction_wf_msg.y = left_gaze_direction_wf.getY();
    lefe_gaze_direction_wf_msg.z = left_gaze_direction_wf.getZ();
    ros_gaze_direction_msg.left_gaze_diection = lefe_gaze_direction_wf_msg;
    geometry_msgs::Vector3 right_gaze_direction_wf_msg;
    right_gaze_direction_wf_msg.x = right_gaze_direction_wf.getX();
    right_gaze_direction_wf_msg.y = right_gaze_direction_wf.getY();
    right_gaze_direction_wf_msg.z = right_gaze_direction_wf.getZ();

    ros_gaze_direction_msg.right_gaze_diection = right_gaze_direction_wf_msg;

    gaze_direction_wf_pub.publish(ros_gaze_direction_msg);
}**/

void system_callback(const sar_core::SystemState::ConstPtr& msg)
{
    int _state = (*msg).system_state;

    if(_state == (*msg).SYSTEM_DOWN){//sar_core::SystemState.SYSTEM_DOWN){//7: SYSTEM_DOWN
        ros::shutdown();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "find_gazepoint");
    ros::Subscriber headposition_sub;
    ros::Subscriber vector_sub;
    ros::Subscriber gaze_direction_sub;
    ros::NodeHandle nh;

    // namespace
    nh.getParam("ns", _namespace);

    // Screen parameters
    nh.getParam("screenAngleInDegrees", screenAngleInDegrees);
    nh.getParam("screenWidth", screenWidth);
    nh.getParam("screenHeight", screenHeight);

    nh.getParam("offset_hfv_wf_z", offset_hfv_wf_z);
    nh.getParam("offset_head_position_cf_z", offset_head_position_cf_z);

    // loading rotation matrix from cf to wf from the parameter server
    // this rotation matrix depends on the rotation of the screen

    // here I load the values in headRotationMatrixCLM_cf of type Matx33d to an array and
    // load the values in the array to a matrix of type tf::Matrix3x3

    // setFromOPenGLSubMatrix (used below) is defined as follows and skips one element i.e. m[3] (the code might have an error)
    // so I use a 12 element array instead of 9, and increment the index by 4 each iteration instead of 3

    // from the source code of setFromOpenGLSubMatrix
    // void setFromOpenGLSubMatrix(const tfScalar *m)
    // {
    //     m_el[0].setValue(m[0],m[4],m[8]);
    //     m_el[1].setValue(m[1],m[5],m[9]);
    //     m_el[2].setValue(m[2],m[6],m[10]);
    // }

    // loading transformation matrix from cf to wf from the parameter server
    nh.getParam("transformation_cf2intermediate_frame", transformation_matrix_cf2intermediate_frame_array_parameter_server);

    for (int i = 0; i <4; i++)
    {
        for(int j =0; j <4; j++)
        {
            transformation_cf2intermediate_frame(i,j) = transformation_matrix_cf2intermediate_frame_array_parameter_server[4*i+j];
        }
    }

    nh.getParam("transformation_intermediate_frame2wf", transformation_matrix_intermediate_frame2wf_array_parameter_server);

    for (int i = 0; i <4; i++)
    {
        for(int j =0; j <4; j++)
        {
            transformation_intermediate_frame2wf(i,j) = transformation_matrix_intermediate_frame2wf_array_parameter_server[4*i+j];
        }
    }

    nh.getParam("transformation_wf2rf", transformation_matrix_wf2rf_array_parameter_server);

    for (int i = 0; i <4; i++)
    {
        for(int j =0; j <4; j++)
        {
            transformation_wf2rf(i,j) = transformation_matrix_wf2rf_array_parameter_server[4*i+j];
        }
    }

    screenAngle = screenAngleInDegrees * M_PI_2 / 90;

    //gaze_point_and_direction_pub = nh.advertise<clm_ros_wrapper::GazePointsAndDirections>(_namespace+"/gaze_points_and_directions", 1);
    headposition_and_gaze_pub = nh.advertise<clm_ros_wrapper::VectorsWithCertaintyWithGazePointsAndDirections>(_namespace+"/head_position_and_eye_gaze", 1);
    gaze_direction_wf_pub = nh.advertise<clm_ros_wrapper::GazeDirections>("/sar/perception/gaze_directions_wf", 1);
    //head_position_rf_pub = nh.advertise<clm_ros_wrapper::VectorsWithCertainty>(_namespace+"/head_positions_rf",1);

    //headposition_sub = nh.subscribe(_namespace+"/head_position", 1, &headposition_callback);
    //vector_sub = nh.subscribe(_namespace+"/head_vector", 1, &vector_callback);
    //gaze_direction_sub = nh.subscribe(_namespace+"/gaze_direction", 1, &gaze_direction_callback);

    headposition_sub = nh.subscribe(_namespace+"/head_positions", 1, &headposition_callback);
    vector_sub = nh.subscribe(_namespace+"/head_vectors", 1, &vector_callback);
    gaze_direction_sub = nh.subscribe(_namespace+"/gaze_directions", 1, &gaze_direction_callback);
    ros::Subscriber system_sub = nh.subscribe("/sar/system/state", 1, &system_callback);
    ros::spin();
}
