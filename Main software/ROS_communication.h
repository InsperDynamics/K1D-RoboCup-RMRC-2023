#pragma once
#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>
using namespace std;
using namespace std::this_thread;
using namespace std::chrono;
using namespace cv;
float current_temperature[768] = {0};
int current_gas = 0;
float cmdvel_linear_x = 0, cmdvel_angular_z = 0;
std_msgs::String opencr_command;
std_msgs::UInt16 opencr_value_1, opencr_value_2;
std_msgs::Header header;
cv_bridge::CvImage img_bridge;
sensor_msgs::Image img_msg;
ros::Publisher pub_command, pub_value_1, pub_value_2, pub_webcam, pub_mattemp, pub_matgas;
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
	sleep_for(seconds(10));
	system("gnome-terminal -- roscore");
	system("gnome-terminal -- roslaunch K1D k1d.launch");
	system("gnome-terminal -- rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200");
	ros::init(argc, argv, "k1d_main");
	ros::NodeHandle nodehandle;
	pub_command = nodehandle.advertise<std_msgs::String>("opencr_command", 1000);
	pub_value_1 = nodehandle.advertise<std_msgs::UInt16>("opencr_value_1", 1000);
	pub_value_2 = nodehandle.advertise<std_msgs::UInt16>("opencr_value_2", 1000);
	sub_temperature = nodehandle.subscribe("temperature", 1000, &temperatureCallback);
	sub_gas = nodehandle.subscribe("gas", 1000, &gasCallback);
	sub_cmdvel = nodehandle.subscribe("cmd_vel", 1000, &cmdvelCallback);
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

void PublishMats(Mat webcam, Mat mattemp, Mat matgas)
{
	header.stamp = ros::Time::now();
	img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, webcam);
	img_bridge.toImageMsg(img_msg);
	pub_webcam.publish(img_msg);
	img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, mattemp);
	img_bridge.toImageMsg(img_msg);
	pub_mattemp.publish(img_msg);
	img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, matgas);
	img_bridge.toImageMsg(img_msg);
	pub_matgas.publish(img_msg);
}