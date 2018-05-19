#include "ros/ros.h"
#include "ros/package.h"
#include "ros/this_node.h"
#include <iostream>
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/Image.h"
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include "control_msgs/JointTrajectoryControllerState.h" //read state of the arm
#include "control_msgs/FollowJointTrajectoryAction.h" //set goal positions
#include "sensor_msgs/JointState.h"
#include "moveit_msgs/GetPositionIK.h"
#include "moveit_msgs/GetPositionFK.h"
#include "moveit_msgs/DisplayRobotState.h"

#include <stdio.h>

using namespace std;
using namespace cv;

/** Global variables */
String face_cascade_name = "haarcascade_frontalface_alt.xml";
String eyes_cascade_name = "haarcascade_eye_tree_eyeglasses.xml";
CascadeClassifier face_cascade;
CascadeClassifier eyes_cascade;
String window_name = "Capture - Face detection";

ros::Publisher ctrlPub;//control the robot
std::vector<double> currentPos = {0,0,0,0,0,0};

/** @function detectAndDisplay */
void detectAndDisplay( Mat frame )
{
    std::vector<Rect> faces;
    Mat frame_gray;

    cvtColor( frame, frame_gray, COLOR_BGR2GRAY );
    equalizeHist( frame_gray, frame_gray );

    //-- Detect faces
    face_cascade.detectMultiScale( frame_gray, faces, 1.1, 2, 0|CASCADE_SCALE_IMAGE, Size(30, 30) );

    for( size_t i = 0; i < faces.size(); i++ )
    {
        Point center( faces[i].x + faces[i].width/2, faces[i].y + faces[i].height/2 );
        ellipse( frame, center, Size( faces[i].width/2, faces[i].height/2), 0, 0, 360, Scalar( 255, 0, 255 ), 4, 8, 0 );

        Mat faceROI = frame_gray( faces[i] );
        std::vector<Rect> eyes;

        //-- In each face, detect eyes
        eyes_cascade.detectMultiScale( faceROI, eyes, 1.1, 2, 0 |CASCADE_SCALE_IMAGE, Size(30, 30) );

        for( size_t j = 0; j < eyes.size(); j++ )
        {
            Point eye_center( faces[i].x + eyes[j].x + eyes[j].width/2, faces[i].y + eyes[j].y + eyes[j].height/2 );
            int radius = cvRound( (eyes[j].width + eyes[j].height)*0.25 );
            circle( frame, eye_center, radius, Scalar( 255, 0, 0 ), 4, 8, 0 );
        }
    }
    if (faces.size()>0){
        control_msgs::FollowJointTrajectoryActionGoal ctrlMsg;
        ctrlMsg.goal.trajectory.joint_names.push_back("shoulder_pan_joint");
        ctrlMsg.goal.trajectory.joint_names.push_back("shoulder_lift_joint");
        ctrlMsg.goal.trajectory.joint_names.push_back("elbow_joint");
        ctrlMsg.goal.trajectory.joint_names.push_back("wrist_1_joint");
        ctrlMsg.goal.trajectory.joint_names.push_back("wrist_2_joint");
        ctrlMsg.goal.trajectory.joint_names.push_back("wrist_3_joint");

        ctrlMsg.goal.trajectory.points.resize(1);
        ctrlMsg.goal.trajectory.points[0].positions.resize(6);
        ctrlMsg.goal.trajectory.points[0].positions[0]=1.0*((320.0-double(faces[0].x + faces[0].width/2))/320.0);
        ctrlMsg.goal.trajectory.points[0].positions[1]=-1.0*((double(faces[0].y + faces[0].height/2))/480.0);
        // Velocities
        ctrlMsg.goal.trajectory.points[0].velocities.resize(6);
        for (size_t jointNo=0; jointNo<6;jointNo++)
            ctrlMsg.goal.trajectory.points[0].velocities[jointNo] = 0.0;
        ctrlMsg.goal.trajectory.points[0].time_from_start = ros::Duration(1.0);

        ctrlPub.publish(ctrlMsg);
    }
    //-- Show what you got
    imshow( window_name, frame );
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg){
    //imshow("image", cv_bridge::toCvShare(msg, "bgr8")->image);
    detectAndDisplay(cv_bridge::toCvShare(msg, "bgr8")->image);
    waitKey(1);
}

void urStateCallback(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg)
{
    for (size_t jointNo=0; jointNo<6;jointNo++){
        ROS_INFO("Joint pos[%d]: %f", (int)jointNo, msg->actual.positions[jointNo]);
        currentPos[jointNo] = msg->actual.positions[jointNo];
    }
}

int main(int argc, char **argv) {
    //initialize node
    ros::init(argc, argv, "cv_example");
    // node handler
    ros::NodeHandle n;

    std::string fpath = ros::package::getPath("opencv_example2");
    //-- 1. Load the cascades
    if( !face_cascade.load( fpath + "/" + face_cascade_name ) ){ printf("--(!)Error loading face cascade\n"); return -1; };
    if( !eyes_cascade.load( fpath + "/" + eyes_cascade_name ) ){ printf("--(!)Error loading eyes cascade\n"); return -1; };

    // subsribe topic
    ros::Subscriber sub = n.subscribe("/cv_camera/image_raw", 10, imageCallback);

    ros::Subscriber subRobot = n.subscribe("/arm_controller/state", 1, urStateCallback);//simulator

    ctrlPub = n.advertise<control_msgs::FollowJointTrajectoryActionGoal>("/arm_controller/follow_joint_trajectory/goal", 1000);

    //ros loop
    ros::spin();
    return 0;
}
