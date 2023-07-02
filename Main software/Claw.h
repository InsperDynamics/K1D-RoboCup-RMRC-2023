#pragma once
#include <iostream>
#include <string>
#include "ROS_communication.h"
using namespace std;

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

void LowerFlipper()
{
  PublishOpenCR("LowerIndividualFlipper", gamepad_value_1, gamepad_value_2);
}

void RaiseFlipper()
{
  PublishOpenCR("RaiseIndividualFlipper", gamepad_value_1, gamepad_value_2);
}

void ClawRetract()
{
  PublishOpenCR("ClawRetract", gamepad_value_1, gamepad_value_2);
}