#pragma once
#include <iostream>
#include <termios.h>
#include <string>
#include <chrono>
#include <thread>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include "open_manipulator_msgs/SetJointPosition.h"
#include "open_manipulator_msgs/SetKinematicsPose.h"
#define PATH_TIME 0.03
#define DELTA 0.005
using namespace std;
using namespace std::this_thread;
using namespace std::chrono;
using namespace cv;
float current_temperature[768] = {0};
float cmdvel_linear_x = 0, cmdvel_angular_z = 0;
bool autonomous_mode = false;
bool dexterity_mode = false;
bool qr_detection = true;
bool hazmat_detection = true;
bool motion_detection = false;
std::vector<double> present_joint_angle;
std::vector<double> present_kinematic_position;
std_msgs::String opencr_command;
std_msgs::UInt16 opencr_value_1, opencr_value_2;
std_msgs::Header header;
cv_bridge::CvImage img_bridge;
sensor_msgs::Image img_msg;
ros::Publisher pub_command, pub_value_1, pub_value_2, pub_webcam, pub_mattemp;
ros::Subscriber sub_temperature, sub_cmdvel, sub_autonomousmode, sub_dexteritymode, sub_qrdetection, sub_hazmatdetection, sub_motiondetection, kinematics_pose_sub, joint_states_sub;
ros::ServiceClient goal_joint_space_path_client_, goal_task_space_path_from_present_position_only_client_, goal_tool_control_client_, goal_task_space_path_from_present_orientation_only_client_;

void temperatureCallback(const std_msgs::Float32MultiArray& temperature)
{
	for (int i=0; i < 64; i++) {
    	current_temperature[i] = temperature.data[i];
  	}
}

void cmdvelCallback(const geometry_msgs::Twist& cmdvel)
{
	cmdvel_linear_x = cmdvel.linear.x;
	cmdvel_angular_z = cmdvel.angular.z;
}

void jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  std::vector<double> temp_angle;
  temp_angle.resize(4);
  for (std::vector<int>::size_type i = 0; i < msg->name.size(); i ++)
  {
    if (!msg->name.at(i).compare("joint1"))  temp_angle.at(0) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("joint2"))  temp_angle.at(1) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("joint3"))  temp_angle.at(2) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("joint4"))  temp_angle.at(3) = (msg->position.at(i));
  }
  present_joint_angle = temp_angle;
}

void kinematicsPoseCallback(const open_manipulator_msgs::KinematicsPose::ConstPtr &msg)
{
  present_kinematic_position = {msg->pose.position.x, msg->pose.position.y, msg->pose.position.z};
}

void autonomousModeCallback(const std_msgs::Bool& autonomous){autonomous_mode = autonomous.data;}
void dexterityModeCallback(const std_msgs::Bool& dexterity){dexterity_mode = dexterity.data;}
void qrDetectionCallback(const std_msgs::Bool& qr){qr_detection = qr.data;}
void hazmatDetectionCallback(const std_msgs::Bool& hazmat){hazmat_detection = hazmat.data;}
void motionDetectionCallback(const std_msgs::Bool& motion){motion_detection = motion.data;}


void ConnectROS(int argc, char** argv)
{
	system("sudo chmod a+rw /dev/ttyACM0");
	system("gnome-terminal -- roscore");
	sleep_for(seconds(10));
	system("gnome-terminal -- roslaunch rosbridge_server rosbridge_websocket.launch");
	system("gnome-terminal -- roslaunch K1D k1d.launch");
	system("gnome-terminal -- rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200");
	ros::init(argc, argv, "k1d_main");
	ros::NodeHandle nodehandle;
	pub_command = nodehandle.advertise<std_msgs::String>("opencr_command", 1000);
	pub_value_1 = nodehandle.advertise<std_msgs::UInt16>("opencr_value_1", 1000);
	pub_value_2 = nodehandle.advertise<std_msgs::UInt16>("opencr_value_2", 1000);
	pub_webcam = nodehandle.advertise<sensor_msgs::Image>("webcam", 1000);
	pub_mattemp = nodehandle.advertise<sensor_msgs::Image>("thermalcam", 1000);
	sub_temperature = nodehandle.subscribe("temperature", 1000, &temperatureCallback);
	sub_cmdvel = nodehandle.subscribe("cmd_vel", 1000, &cmdvelCallback);
	sub_autonomousmode = nodehandle.subscribe("autonomous_mode", 1000, &autonomousModeCallback);
	sub_dexteritymode = nodehandle.subscribe("dexterity_mode", 1000, &dexterityModeCallback);
	sub_qrdetection = nodehandle.subscribe("qr_detection", 1000, &qrDetectionCallback);
	sub_hazmatdetection = nodehandle.subscribe("hazmat_detection", 1000, &hazmatDetectionCallback);
	sub_motiondetection = nodehandle.subscribe("motion_detection", 1000, &motionDetectionCallback);
	joint_states_sub = nodehandle.subscribe("joint_states", 1000, &jointStatesCallback);
  kinematics_pose_sub = nodehandle.subscribe("/gripper/kinematics_pose", 1000, &kinematicsPoseCallback);
	goal_joint_space_path_client_ = nodehandle.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_joint_space_path");
  goal_task_space_path_from_present_position_only_client_ = nodehandle.serviceClient<open_manipulator_msgs::SetKinematicsPose>("goal_task_space_path_from_present_position_only");
  goal_tool_control_client_ = nodehandle.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_tool_control");
	goal_task_space_path_from_present_orientation_only_client_ = nodehandle.serviceClient<open_manipulator_msgs::SetKinematicsPose>("goal_task_space_path_from_present_orientation_only");
}

bool setJointSpacePath(std::vector<std::string> joint_name, std::vector<double> joint_angle, double 0.5)
{
  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.joint_name = joint_name;
  srv.request.joint_position.position = joint_angle;
  srv.request.0.5 = 0.5;

  if (goal_joint_space_path_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

bool setToolControl(std::vector<double> joint_angle)
{
  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.joint_name.push_back(priv_node_handle_.param<std::string>("end_effector_name", "gripper"));
  srv.request.joint_position.position = joint_angle;

  if (goal_tool_control_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

bool setTaskSpacePathFromPresentPositionOnly(std::vector<double> kinematics_pose, double 0.5)
{
  open_manipulator_msgs::SetKinematicsPose srv;
  srv.request.planning_group = priv_node_handle_.param<std::string>("end_effector_name", "gripper");
  srv.request.kinematics_pose.pose.position.x = kinematics_pose.at(0);
  srv.request.kinematics_pose.pose.position.y = kinematics_pose.at(1);
  srv.request.kinematics_pose.pose.position.z = kinematics_pose.at(2);
  srv.request.0.5 = 0.5;

  if (goal_task_space_path_from_present_position_only_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

bool setTaskSpacePathFromPresentOrientationOnly(std::vector<double> kinematics_pose)
{
  open_manipulator_msgs::SetKinematicsPose srv;
  double x  = sin(kinematics_pose.at(0)/2) * cos(kinematics_pose.at(1)/2) * cos(kinematics_pose.at(2)/2) - cos(kinematics_pose.at(0)/2) * sin(kinematics_pose.at(1)/2) * sin(kinematics_pose.at(2)/2);
  double y = cos(kinematics_pose.at(0)/2) * sin(kinematics_pose.at(1)/2) * cos(kinematics_pose.at(2)/2) + sin(kinematics_pose.at(0)/2) * cos(kinematics_pose.at(1)/2) * sin(kinematics_pose.at(2)/2);
  double w = cos(kinematics_pose.at(0)/2) * cos(kinematics_pose.at(1)/2) * cos(kinematics_pose.at(2)/2) + sin(kinematics_pose.at(0)/2) * sin(kinematics_pose.at(1)/2) * sin(kinematics_pose.at(2)/2);
  double z = cos(kinematics_pose.at(0)/2) * cos(kinematics_pose.at(1)/2) * sin(kinematics_pose.at(2)/2) - sin(kinematics_pose.at(0)/2) * sin(kinematics_pose.at(1)/2) * cos(kinematics_pose.at(2)/2);
  srv.request.planning_group = "gripper";
  srv.request.kinematics_pose.pose.orientation.w = w;
  srv.request.kinematics_pose.pose.orientation.x = x;
  srv.request.kinematics_pose.pose.orientation.y = y;
  srv.request.kinematics_pose.pose.orientation.z = z;
  srv.request.path_time = PATH_TIME;

  if (goal_task_space_path_from_present_orientation_only_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

void ClawBackward()
{
  std::vector<double> goal {0.0, 0.0, DELTA};
  setTaskSpacePathFromPresentPositionOnly(goal);
}

void ClawForward()
{
  std::vector<double> goal {0.0, 0.0, -DELTA};
  setTaskSpacePathFromPresentPositionOnly(goal);
}

void ClawDown()
{
  std::vector<double> goal {DELTA, 0.0, 0.0};
  setTaskSpacePathFromPresentPositionOnly(goal); 
}

void ClawUp()
{
  std::vector<double> goal {-DELTA, 0.0, 0.0};
  setTaskSpacePathFromPresentPositionOnly(goal); 
}

void ClawLeft()
{
  std::vector<double> goal {0.0, DELTA, 0.0};
  setTaskSpacePathFromPresentPositionOnly(goal); 
}

void ClawRight()
{
  std::vector<double> goal {0.0, -DELTA, 0.0};
  setTaskSpacePathFromPresentPositionOnly(goal); 
}

void ClawOpen()
{
  std::vector<double> joint_angle;
  joint_angle.push_back(0.01);
  setToolControl(joint_angle);
}

void ClawClose()
{
  std::vector<double> joint_angle;
  joint_angle.push_back(-0.01);
  setToolControl(joint_angle);
}

void GoToPreset(vector<double> angles)
{
	vector<string> name = {"joint1","joint2","joint3","joint4"};
	setJointSpacePath(name, angles);
}

void ReadOpenCR() 
{
	ros::spinOnce();
	sleep_for(microseconds(1000));
}

void PublishOpenCR(string command, int value_1, int value_2)
{
	opencr_command.data = command;
	opencr_value_1.data = value_1;
	opencr_value_2.data = value_2;
	pub_command.publish(opencr_command);
	pub_value_1.publish(opencr_value_1);
	pub_value_2.publish(opencr_value_2);
}

void PublishMats(Mat webcam, Mat mattemp)
{
	header.stamp = ros::Time::now();
	img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, webcam);
	img_bridge.toImageMsg(img_msg);
	pub_webcam.publish(img_msg);
	img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, mattemp);
	img_bridge.toImageMsg(img_msg);
	pub_mattemp.publish(img_msg);
}