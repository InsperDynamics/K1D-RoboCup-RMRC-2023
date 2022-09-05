#pragma once
#include <iostream>
#include <math.h>
#include "SDL2/SDL.h"
#define JOYSTICK_DEAD_ZONE 4000
#define JOYSTICK_MAXIMUM_ZONE 33000
using namespace std;
static SDL_Joystick* gGameController = NULL;
static SDL_Event sdl_event;
static int max_pwm = 250;
static int xAnalog_left = 0;
static int yAnalog_left = 0;
string gamepad_command = "";
int gamepad_value_1 = 0;
int gamepad_value_2 = 0;
bool autonomous_mode = false;
bool dexterity_mode = false;
bool invert_camera = false;


void InitializeGamepad()
{
	SDL_Init(SDL_INIT_VIDEO | SDL_INIT_JOYSTICK);
    gGameController = SDL_JoystickOpen(0);
}


void UpdateAnalog()
{
	while (SDL_PollEvent(&sdl_event) != 0)
    {
		if (sdl_event.type == SDL_JOYBUTTONDOWN)
		{
			int buttonId = sdl_event.jbutton.button;
			switch (buttonId)
			{
			case 0:
				gamepad_command = "dexterity_mode";
				return;
				break;
			case 1:
				gamepad_command = "motion_detection";
				return;
				break;
			case 2:
				gamepad_command = "qr_detection";
				return;
				break;
			case 3:
				gamepad_command = "hazmat_detection";
				return;
				break;
			case 4:
			case 5:
				gamepad_command = "invert_camera";
				return;
				break;
			case 6:
			case 7:
				gamepad_command = "autonomous_mode";
				return;
				break;
			}
		}
		else if (sdl_event.type == SDL_JOYAXISMOTION)
        {
			float pwm_to_joystick_zone = JOYSTICK_MAXIMUM_ZONE / max_pwm;
			switch (sdl_event.jaxis.axis)
			{
			case 0:
				if (abs(sdl_event.jaxis.value) > abs(JOYSTICK_DEAD_ZONE))
					xAnalog_left = sdl_event.jaxis.value / pwm_to_joystick_zone;
				else
					xAnalog_left = 0;
				break;
			case 1:
				if (abs(sdl_event.jaxis.value) > abs(JOYSTICK_DEAD_ZONE))
					yAnalog_left = -(sdl_event.jaxis.value / pwm_to_joystick_zone);
				else
					yAnalog_left = 0;
				break;
			}
        }
    }
}


void UpdateGamepadInput()
{
	float magnitude = sqrt(xAnalog_left * xAnalog_left + yAnalog_left * yAnalog_left);
	int pwm = static_cast<int>(0.000042*pow(magnitude, 3) - 0.017060*pow(magnitude, 2) + 2.810082*magnitude - 56.496350);
	if (pwm > max_pwm)
		pwm = max_pwm;
	else if (pwm < 0)
		pwm = 0;
	double theta = atan2(yAnalog_left, xAnalog_left);
	if (theta < 0)
		theta += 2 * M_PI;
	if (xAnalog_left == 0 && yAnalog_left == 0)
	{
		gamepad_command = "MotorsStop";
	}
	else 
	{
		gamepad_command = "MotorsMove";
		if (xAnalog_left >= 0 && yAnalog_left >= 0)
		{
			gamepad_value_1 = int(pwm);
			gamepad_value_2 = int(pwm*((4*theta/M_PI) - 1));
		}
		else if (xAnalog_left < 0 && yAnalog_left >= 0)
		{
			gamepad_value_1 = int(pwm*(1 - (4*(theta-(M_PI/2))/M_PI)));
			gamepad_value_2 = int(pwm);
		}
		else if (xAnalog_left < 0 && yAnalog_left < 0)
		{
			gamepad_value_1 = -int(pwm);
			gamepad_value_2 = int(pwm*(1 - (4*(theta-M_PI)/M_PI)));
		}
		else 
		{
			gamepad_value_1 = -int(pwm*(1 - (4*(theta-(3*M_PI/2))/M_PI)));
			gamepad_value_2 = -int(pwm);
		}
	}
	UpdateAnalog();
}