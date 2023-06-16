#pragma once
#include <iostream>
#include <string>
#include "ROS_communication.h"
#include <moveit/planning_scene_interface/planning_scene_interface.h>
using namespace std;
#define DELTA 18
#define MDELTA 0.005
#define FLIPPER_DELTA 30
#define GRIPPER_DELTA 20

geometry_msgs::PoseStamped targetPose;
vector<double> jointstate;

void GripperForward() 
{
    targetPose = p_move->getCurrentPose();
    targetPose.pose.position.x += MDELTA;
    p_move->setPoseTarget(targetPose);

    if (p_move->plan(*p_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      
      p_move->getJointValueTarget(jointstate);
      PublishJointsOpenCR(jointstate);
    }
}

void GripperBackward() 
{
    targetPose = p_move->getCurrentPose();
    targetPose.pose.position.x -= MDELTA;
    p_move->setPoseTarget(targetPose);

    if (p_move->plan(*p_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      
      p_move->getJointValueTarget(jointstate);
      PublishJointsOpenCR(jointstate);
    }
}

void GripperUp() 
{
    targetPose = p_move->getCurrentPose();
    targetPose.pose.position.z += MDELTA;
    p_move->setPoseTarget(targetPose);

    if (p_move->plan(*p_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      
      p_move->getJointValueTarget(jointstate);
      PublishJointsOpenCR(jointstate);
    }
}

void GripperDown() 
{
    targetPose = p_move->getCurrentPose();
    targetPose.pose.position.z -= MDELTA;
    p_move->setPoseTarget(targetPose);

    if (p_move->plan(*p_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      
      p_move->getJointValueTarget(jointstate);
      PublishJointsOpenCR(jointstate);
    }
}

void FirstPlus()
{
	PublishOpenCR("First+", DELTA, 0);	
}

void FirstMinus()
{
	PublishOpenCR("First-", DELTA, 0);	
}

void SecondPlus()
{
  PublishOpenCR("Second+", DELTA, 0);
}

void SecondMinus()
{
  PublishOpenCR("Second-", DELTA, 0);
}

void ThirdPlus()
{
  PublishOpenCR("Third+", DELTA, 0);
}

void ThirdMinus()
{
  PublishOpenCR("Third-", DELTA, 0);
}

void ClawOpen()
{
  PublishOpenCR("OpenGripper", GRIPPER_DELTA, 0);
}

void ClawClose()
{
  PublishOpenCR("CloseGripper", GRIPPER_DELTA, 0);
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
	PublishJointsOpenCR(angles);
}