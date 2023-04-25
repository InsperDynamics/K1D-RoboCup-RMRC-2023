#pragma once
#include <iostream>
#include <string>
#include "ROS_communication.h"
using namespace std;
#define DELTA 1
#define FLIPPER_DELTA 30
#define GRIPPER_DELTA 1

void ClawBackward()
{
	for (int i = 0; i < 3; i++)
	{
		joint_msg.data[i] = present_joint_angle.at(i);
  	}
	joint_msg.data[0] -= DELTA;
	pub_goal_joints.publish(joint_msg);
}

void ClawForward()
{
	for (int i = 0; i < 3; i++)
	{
		joint_msg.data[i] = present_joint_angle.at(i);
  	}
	joint_msg.data[0] += DELTA;
	pub_goal_joints.publish(joint_msg);	
}

void ClawDown()
{
	for (int i = 0; i < 3; i++)
	{
		joint_msg.data[i] = present_joint_angle.at(i);
  	}
	joint_msg.data[1] -= DELTA;
	pub_goal_joints.publish(joint_msg);
}

void ClawUp()
{
	for (int i = 0; i < 3; i++)
	{
		joint_msg.data[i] = present_joint_angle.at(i);
  	}
	joint_msg.data[1] += DELTA;
	pub_goal_joints.publish(joint_msg);
}

void ClawLeft()
{
	for (int i = 0; i < 3; i++)
	{
		joint_msg.data[i] = present_joint_angle.at(i);
  	}
	joint_msg.data[2] -= DELTA;
	pub_goal_joints.publish(joint_msg);
}

void ClawRight()
{
  	for (int i = 0; i < 3; i++)
	{
		joint_msg.data[i] = present_joint_angle.at(i);
  	}
	joint_msg.data[2] += DELTA;
	pub_goal_joints.publish(joint_msg);
}

void ClawOpen()
{
  PublishOpenCR("MoveGripper", GRIPPER_DELTA, 0);
}

void ClawClose()
{
  PublishOpenCR("MoveGripper", -GRIPPER_DELTA, 0);
}

void RaiseFrontFlippers()
{
  PublishOpenCR("RaiseFrontFlippers", FLIPPER_DELTA, 0);
}

void LowerFrontFlippers()
{
  PublishOpenCR("LowerFrontFlippers", FLIPPER_DELTA, 0);
}

void RaiseBackFlippers()
{
  PublishOpenCR("RaiseBackFlippers", FLIPPER_DELTA, 0);
}

void LowerBackFlippers()
{
  PublishOpenCR("LowerBackFlippers", FLIPPER_DELTA, 0);
}

void ClawRetract()
{
  
}

void GoToPreset(vector<double> angles)
{
	
}