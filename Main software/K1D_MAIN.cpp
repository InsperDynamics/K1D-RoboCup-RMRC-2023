#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include "ROS_communication.h"
#include "Gamepad_controller.h"
#include "Claw.h"
#include "Thermal_gas.h"
#include "QR_read.h"
#include "Motion_detection.h"
#include "Hazmat_detection.h"
using namespace std;
using namespace cv;
VideoCapture capture;
VideoCapture captureClaw;
Mat webcam_image;
Mat webcam_imageClaw;
int current_camera = 1;


void openCamera(int index)
{
	//1 = xiaomi_forwards, 2 = xiaomi_backwards, 3 = D435i
	current_camera = index;
}


void checkUserInput()
{
	if (gamepad_command == "autonomous_mode")
		autonomous_mode = !autonomous_mode;
	else if (gamepad_command == "dexterity_mode")
	{
		dexterity_mode = !dexterity_mode;
		cout << "DEXTERITY: " << to_string(dexterity_mode) << "\n";
		if (dexterity_mode)
			openCamera(3);
		else
			openCamera(1);
	}
	else if (gamepad_command == "turn_camera")
	{
		if (current_camera == 1)
			openCamera(2);
		else if (current_camera == 2)
			openCamera(1);
	}
	else if (gamepad_command == "motion_detection")
		motion_detection = !motion_detection;
	else if (gamepad_command == "qr_detection")
		qr_detection = !qr_detection;
	else if (gamepad_command == "hazmat_detection")
		hazmat_detection = !hazmat_detection;
	//condition for claw goes here
	else if (!autonomous_mode && !gamepad_command.empty())
	{
		if (current_camera == 2)
		{
			int temp = gamepad_value_1;	
			gamepad_value_1 = -gamepad_value_2;
			gamepad_value_2 = -temp;
		}
		cout << gamepad_command << " " << to_string(gamepad_value_1) << " " << to_string(gamepad_value_2) << "\n";
		PublishOpenCR(gamepad_command, gamepad_value_1, gamepad_value_2);
	} 
	else if (autonomous_mode)
	{
		int pwm_l = 0;
		int pwm_r = 0;
		//autonomous mode pwm logic goes here
		cout << "(AUTONOMOUS) MotorsMove " << to_string(pwm_l) << " " << to_string(pwm_r) << "\n";
		PublishOpenCR("MotorsMove", pwm_l, pwm_r);
	}
}


void checkSensorsFeed()
{
	ReadOpenCR();
	UpdateGas(current_gas);
	UpdateThermal(current_temperature);
	//frame from camera to Mat using >> operator
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
	openCamera(1);
	InitializeQR();
	InitializeHazmat();
}


void loop()
{
	if (SDL_NumJoysticks() < 1)
	{
		try
		{
			InitializeGamepad();
		}
		catch(const std::exception& e)
		{
			std::cerr << e.what() << '\n';
		}
	}
	UpdateGamepadInput();
	checkUserInput();
	checkSensorsFeed();
	PublishMats();
	waitKey(1);
}


int main(int argc, char** argv)
{
	setup(argc, argv);
	while (true)
	{
		try
		{
			loop();
		}
		catch (const exception& e)
		{
			cout << e.what() << endl;
		}
		catch ( ros::Exception &e )
		{
			cout << e.what() << endl;
		}
		catch (const cv::Exception& e)
		{
			cout << e.what() << endl;
		}
		catch (...)
		{
			cout << "Unknown error occurred!" << endl;
		}
	}
	return 0;
}