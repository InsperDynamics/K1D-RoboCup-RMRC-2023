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
int current_gas = 0;
float cmdvel_linear_x = 0, cmdvel_angular_z = 0;
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
ros::Subscriber sub_temperature, sub_gas, sub_cmdvel;

void temperatureCallback(const std_msgs::Float32MultiArray& temperature)
{
	for (int i=0; i < 64; i++) {
    	current_temperature[i] = temperature.data[i];
  	}
}

void gasCallback(const std_msgs::UInt16& gas)
{
	current_gas = gas.data;
}

void cmdvelCallback(const geometry_msgs::Twist& cmdvel)
{
	cmdvel_linear_x = cmdvel.linear.x;
	cmdvel_angular_z = cmdvel.angular.z;
}


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
	pub_command = nodehandle.advertise<std_msgs::String>("arduino_command", 1);
	pub_value_1 = nodehandle.advertise<std_msgs::Int16>("arduino_value_1", 1);
	pub_value_2 = nodehandle.advertise<std_msgs::Int16>("arduino_value_2", 1);
	pub_webcam = nodehandle.advertise<std_msgs::String>("webcam", 1);
	pub_mattemp = nodehandle.advertise<std_msgs::String>("thermalcam", 1);
  	pub_goal_joints = nodehandle.advertise<std_msgs::Int32MultiArray>("goal_joints", 1);
	sub_temperature = nodehandle.subscribe("temperature", 1, &temperatureCallback);
	sub_gas = nodehandle.subscribe("gas", 1, &gasCallback);
	sub_cmdvel = nodehandle.subscribe("cmd_vel", 1, &cmdvelCallback);
}

void ReadOpenCR() 
{
	ros::spinOnce();
  	ros::Rate r(60);
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