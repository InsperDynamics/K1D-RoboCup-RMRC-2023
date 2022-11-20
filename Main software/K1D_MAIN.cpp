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

void MoveManual()
{
	if (invert_camera)
	{
		int temp = gamepad_value_1;	
		gamepad_value_1 = -gamepad_value_2;
		gamepad_value_2 = -temp;
	}
	cout << gamepad_command << " " << to_string(gamepad_value_1) << " " << to_string(gamepad_value_2) << "\n";
	PublishOpenCR(gamepad_command, gamepad_value_1, gamepad_value_2);
}

void MoveAutonomous()
{
	int pwm_l = 0;
	int pwm_r = 0;
	//autonomous mode pwm logic goes here
	cout << "(AUTONOMOUS) MotorsMove " << to_string(pwm_l) << " " << to_string(pwm_r) << "\n";
	PublishOpenCR("MotorsMove", pwm_l, pwm_r);
}

void checkUserInput()
{
	if (gamepad_command == "autonomous_mode")
		autonomous_mode = !autonomous_mode;
	else if (gamepad_command == "dexterity_mode")
		dexterity_mode = !dexterity_mode;
	else if (gamepad_command == "motion_detection")
		motion_detection = !motion_detection;
	else if (gamepad_command == "qr_detection")
		qr_detection = !qr_detection;
	else if (gamepad_command == "hazmat_detection")
		hazmat_detection = !hazmat_detection;
	else if (gamepad_command == "invert_camera")
		invert_camera = !invert_camera;
	//condition for claw goes here
	else if (!autonomous_mode && !gamepad_command.empty())
		MoveManual();
	else if (autonomous_mode)
		MoveAutonomous();
}

void checkSensorsFeed()
{
	ReadOpenCR();
	UpdateThermal(current_temperature);
	captureA >> frameA;
	captureB >> frameB;
	resize(frameA, frameA, Size(resolution_horizontal, resolution_vertical), INTER_NEAREST);
	resize(frameB, frameB, Size(resolution_horizontal, resolution_vertical), INTER_NEAREST);
	hconcat(frameA, frameB, webcam_image);
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