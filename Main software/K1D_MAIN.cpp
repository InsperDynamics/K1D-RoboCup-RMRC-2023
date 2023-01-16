#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include "ROS_communication.h"
#include "Gamepad_controller.h"
#include "Claw.h"
#include "Thermal.h"
#include "QR_read.h"
#include "Motion_detection.h"
#include "Hazmat_detection.h"
using namespace std;
using namespace cv;
int resolution_horizontal = 640;
int resolution_vertical = 480;
VideoCapture captureA;
VideoCapture captureB;
Mat frameA = Mat::zeros(resolution_horizontal, resolution_vertical, CV_8UC3);
Mat frameB = Mat::zeros(resolution_horizontal, resolution_vertical, CV_8UC3);
Mat webcam_image = Mat::zeros(2 * resolution_horizontal, resolution_vertical, CV_8UC3);

void openCamera()
{
	captureA.open(2);
	captureB.open(4);
	captureA.set(CAP_PROP_FPS, 30);
	captureB.set(CAP_PROP_FPS, 30);
}

float mapPwm(float x, float out_min, float out_max)
{
  return x * (out_max - out_min) + out_min;
}

void MoveManual()
{
	cout << gamepad_command << " " << to_string(gamepad_value_1) << " " << to_string(gamepad_value_2) << "\n";
	PublishOpenCR(gamepad_command, gamepad_value_1, gamepad_value_2);
}

void MoveAutonomous()
{
	const float wheelBase = 0.12;
	const float wheelRadius = 0.055;
	//value caps at [-1, 1]
  	float x = max(min(cmdvel_linear_x, 1.0f), -1.0f);
  	float z = max(min(cmdvel_angular_z, 1.0f), -1.0f);
	float l = ((2*msg.linear.x) - (msg.angular.z*wheelBase)) / (2*wheelRadius);
  	float r = ((2*msg.linear.x) + (msg.angular.z*wheelBase)) / (2*wheelRadius);
	uint16_t pwm_l = mapPwm(fabs(l), 50, 250);
	uint16_t pwm_r = mapPwm(fabs(r), 50, 250);
	cout << "(AUTONOMOUS) MotorsMove " << to_string(pwm_l) << " " << to_string(pwm_r) << "\n";
	PublishOpenCR("MotorsMove", pwm_l, pwm_r);
}

void checkUserInput()
{
	if (gamepad_command == "autonomous_mode")
		autonomous_mode = !autonomous_mode;
	else if (gamepad_command == "dexterity_mode")
	{
		if (dexterity_mode)
			ClawRetract();
		dexterity_mode = !dexterity_mode;
	}
	else if (gamepad_command == "motion_detection")
		motion_detection = !motion_detection;
	else if (gamepad_command == "qr_detection")
		qr_detection = !qr_detection;
	else if (gamepad_command == "hazmat_detection")
		hazmat_detection = !hazmat_detection;
	else if (gamepad_command == "RaiseFrontFlippers")
		RaiseFrontFlippers();
	else if (gamepad_command == "LowerFrontFlippers")
		LowerFrontFlippers();
	else if (gamepad_command == "RaiseBackFlippers")
		RaiseBackFlippers();
	else if (gamepad_command == "LowerBackFlippers")
		LowerBackFlippers();
	else if (gamepad_command == "ClawUp" && dexterity_mode)
		ClawUp();
	else if (gamepad_command == "ClawDown" && dexterity_mode)
		ClawDown();
	else if (gamepad_command == "ClawForward" && dexterity_mode)
		ClawForward();
	else if (gamepad_command == "ClawBackward" && dexterity_mode)
		ClawBackward();
	else if (gamepad_command == "ClawOpen" && dexterity_mode)
		ClawOpen();
	else if (gamepad_command == "ClawClose" && dexterity_mode)
		ClawClose();
	else if (!autonomous_mode && !gamepad_command.empty())
		MoveManual();
	else if (autonomous_mode)
		MoveAutonomous();
}

void captureFrame()
{
	captureA >> frameA;
	captureB >> frameB;
	resize(frameA, frameA, Size(resolution_horizontal, resolution_vertical), INTER_NEAREST);
	resize(frameB, frameB, Size(resolution_horizontal, resolution_vertical), INTER_NEAREST);
	hconcat(frameA, frameB, webcam_image);
}

void checkSensorsFeed()
{
	ReadOpenCR();
	UpdateThermal(current_temperature);
	captureFrame();
	if (qr_detection)
		webcam_image = ReadQR(webcam_image);
	if (hazmat_detection)
		webcam_image = DetectHazmat(webcam_image);
	if (motion_detection)
		webcam_image = DetectMotion(webcam_image);
}


void setup(int argc, char** argv) 
{
	system("gnome-terminal -- play '|rec --buffer 512 -d'");
	ConnectROS(argc, argv);
	openCamera();
	InitializeQR();
	InitializeHazmat();
}


void loop()
{
	if (SDL_NumJoysticks() < 1)
		InitializeGamepad();
	UpdateGamepadInput();
	checkUserInput();
	checkSensorsFeed();
	PublishMats(webcam_image, thermal_image);
	imshow("K1D", webcam_image);
	waitKey(1);
}


int main(int argc, char** argv)
{
	setup(argc, argv);
	while (true)
	{
		try {loop();}
		catch (const std::exception& e){cout << e.what() << endl;}
		catch (ros::Exception &e ){cout << e.what() << endl;}
		catch (const cv::Exception& e){cout << e.what() << endl;}
		catch (...){cout << "Unknown error occurred!" << endl;}
	}
	return 0;
}