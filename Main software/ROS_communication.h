#pragma once
#include <iostream>
#include <termios.h>
#include <string>
#include <chrono>
#include <thread>
#include <ros/ros.h>
#include "base64.cpp"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
using namespace std;
using namespace std::this_thread;
using namespace std::chrono;
using namespace cv;
float current_temperature[64] = {0};
float cmdvel_linear_x = 0, cmdvel_angular_z = 0;
bool autonomous_mode = false;
bool dexterity_mode = false;
bool qr_detection = false;
bool hazmat_detection = false;
bool motion_detection = false;
std::vector<double> present_joint_angle;
std_msgs::String arduino_command;
std_msgs::Int16 arduino_value_1, arduino_value_2;
std_msgs::Header header;
std_msgs::Int32MultiArray joint_msg;
cv_bridge::CvImage img_bridge;
std::vector<uchar> to_str_buf;
std_msgs::String img_msg;
std::string encoded;
ros::Publisher pub_command, pub_value_1, pub_value_2, pub_webcam, pub_mattemp, pub_goal_joints;
ros::Subscriber sub_temperature, sub_cmdvel, sub_autonomousmode, sub_dexteritymode, sub_qrdetection, sub_hazmatdetection, sub_motiondetection, joint_states_sub;

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
  temp_angle.resize(3);
  for (int i = 0; i < 3; i++)
  {
	temp_angle.at(i) = (msg->position.at(i));
  }
  present_joint_angle = temp_angle;
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
	system("gnome-terminal -- rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200");
	system("gnome-terminal -- roslaunch k1d k1d.launch");
	ros::init(argc, argv, "k1d_main");
	ros::NodeHandle nodehandle;
	pub_command = nodehandle.advertise<std_msgs::String>("arduino_command", 1000);
	pub_value_1 = nodehandle.advertise<std_msgs::Int16>("arduino_value_1", 1000);
	pub_value_2 = nodehandle.advertise<std_msgs::Int16>("arduino_value_2", 1000);
	pub_webcam = nodehandle.advertise<std_msgs::String>("webcam", 1000);
	pub_mattemp = nodehandle.advertise<std_msgs::String>("thermalcam", 1000);
  	pub_goal_joints = nodehandle.advertise<std_msgs::Int32MultiArray>("goal_joints", 1000);
	sub_temperature = nodehandle.subscribe("temperature", 1000, &temperatureCallback);
	sub_cmdvel = nodehandle.subscribe("cmd_vel", 1000, &cmdvelCallback);
	sub_autonomousmode = nodehandle.subscribe("autonomous_mode", 1000, &autonomousModeCallback);
	sub_dexteritymode = nodehandle.subscribe("dexterity_mode", 1000, &dexterityModeCallback);
	sub_qrdetection = nodehandle.subscribe("qr_detection", 1000, &qrDetectionCallback);
	sub_hazmatdetection = nodehandle.subscribe("hazmat_detection", 1000, &hazmatDetectionCallback);
	sub_motiondetection = nodehandle.subscribe("motion_detection", 1000, &motionDetectionCallback);
	joint_states_sub = nodehandle.subscribe("joint_states", 1000, &jointStatesCallback);
}

void ReadOpenCR() 
{
	ros::spinOnce();
  	ros::Rate r(10);
	r.sleep();
}

void PublishOpenCR(string command, int value_1, int value_2)
{
	arduino_command.data = command;
	arduino_value_1.data = value_1;
	arduino_value_2.data = value_2;
	pub_command.publish(arduino_command);
	pub_value_1.publish(arduino_value_1);
	pub_value_2.publish(arduino_value_2);
}

void PublishMats(Mat webcam, Mat mattemp)
{
    cv::imencode(".jpg", webcam, to_str_buf);
    auto *enc_msg = reinterpret_cast<unsigned char*>(to_str_buf.data());
    encoded = base64_encode(enc_msg, to_str_buf.size());
	img_msg.data = encoded;
	pub_webcam.publish(img_msg);

	cv::imencode(".jpg", mattemp, to_str_buf);
    enc_msg = reinterpret_cast<unsigned char*>(to_str_buf.data());
    encoded = base64_encode(enc_msg, to_str_buf.size());
	img_msg.data = encoded;
	pub_mattemp.publish(img_msg);
}