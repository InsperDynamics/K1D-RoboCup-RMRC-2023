#pragma once
#include <iostream>
#include <string>
#include "ROS_communication.h"
using namespace std;

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

void RaiseFrontFlippers()
{
  std::vector<double> goal {0.0, -DELTA, 0.0};
  setTaskSpacePathFromPresentPositionOnly(goal); 
}

void LowerFrontFlippers()
{
  std::vector<double> goal {0.0, -DELTA, 0.0};
  setTaskSpacePathFromPresentPositionOnly(goal); 
}

void RaiseBackFlippers()
{
  std::vector<double> goal {0.0, -DELTA, 0.0};
  setTaskSpacePathFromPresentPositionOnly(goal); 
}

void LowerBackFlippers()
{
  std::vector<double> goal {0.0, -DELTA, 0.0};
  setTaskSpacePathFromPresentPositionOnly(goal); 
}

void ClawRetract()
{
  std::vector<double> goal {0.0, -DELTA, 0.0};
  setTaskSpacePathFromPresentPositionOnly(goal); 
}

void GoToPreset(vector<double> angles)
{
	vector<string> name = {"joint1","joint2","joint3","joint4"};
	setJointSpacePath(name, angles);
}