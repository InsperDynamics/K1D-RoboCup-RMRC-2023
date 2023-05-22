#pragma once
#include <iostream>
#include <math.h>
#include <SDL2/SDL.h>
#define JOYSTICK_DEAD_ZONE 4000
#define JOYSTICK_MAXIMUM_ZONE 33000
using namespace std;
static SDL_GameController* gGameController = NULL;
static SDL_Event sdl_event;
static int max_pwm = 250;
static int min_pwm = 40;
static int xAnalog_left = 0;
static int yAnalog_left = 0;
string gamepad_command = "";
int gamepad_value_1 = 0;
int gamepad_value_2 = 0;
bool isPressed = false;
SDL_GameControllerAxis lastAxis = SDL_CONTROLLER_AXIS_INVALID;

map<Uint8,vector<double>> preset;
int dir = 1;

void SetHomePreset()
{
	std::vector<double> joint_angle;

	joint_angle.push_back(0.0);
    joint_angle.push_back(0.0);
    joint_angle.push_back(0.0);
    joint_angle.push_back(0.0);

	preset[SDL_CONTROLLER_BUTTON_X] = joint_angle;

}

void InitializeGamepad()
{
	SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMECONTROLLER);
    gGameController = SDL_GameControllerOpen(0);
	SetHomePreset();
}

void UpdateRawInput()
{	
	while(SDL_PollEvent(&sdl_event))
	{
		if (sdl_event.type == SDL_CONTROLLERBUTTONUP)
		{
			isPressed = false;
		} 

		if (SDL_GameControllerGetAxis(gGameController, lastAxis) < JOYSTICK_DEAD_ZONE)
		{
			lastAxis = SDL_CONTROLLER_AXIS_INVALID;
		}

		if (sdl_event.cbutton.type == SDL_CONTROLLERBUTTONDOWN)
		{
			isPressed = true;

			if (sdl_event.cbutton.button != SDL_CONTROLLER_BUTTON_BACK && SDL_GameControllerGetButton(gGameController, SDL_CONTROLLER_BUTTON_BACK))
			{  
				if (preset.count(sdl_event.cbutton.button))
				{
					GoToPreset(preset[sdl_event.cbutton.button]);
					cout << "enviado\n";
				}
				break;
			}

			if (sdl_event.cbutton.button != SDL_CONTROLLER_BUTTON_START && SDL_GameControllerGetButton(gGameController, SDL_CONTROLLER_BUTTON_START))
			{  
				preset[sdl_event.cbutton.button] = present_joint_angle;
				cout << "Salvo\n";
				break;
			}

			switch (sdl_event.cbutton.button)
			{
				case SDL_CONTROLLER_BUTTON_RIGHTSHOULDER:
					gamepad_command = "RaiseFrontFlippers";
					break;
				case SDL_CONTROLLER_BUTTON_LEFTSHOULDER:
				    gamepad_command = "RaiseBackFlippers";
                    break;
				case SDL_CONTROLLER_BUTTON_Y:
					gamepad_command = "Third+";
					break;
				case SDL_CONTROLLER_BUTTON_A:
					gamepad_command = "Third-";
                    break;
				case SDL_CONTROLLER_BUTTON_DPAD_UP:
					gamepad_command = "Second+";
                    break;
				case SDL_CONTROLLER_BUTTON_DPAD_DOWN:
					gamepad_command = "Second-";
                    break;
				case SDL_CONTROLLER_BUTTON_DPAD_RIGHT:
					gamepad_command = "First+";
                    break;
				case SDL_CONTROLLER_BUTTON_DPAD_LEFT:
					gamepad_command = "First-";
                    break;
				case SDL_CONTROLLER_BUTTON_X:
                    gamepad_command = "ClawOpen";
				    break;
                case SDL_CONTROLLER_BUTTON_B:
				    gamepad_command = "ClawClose";
                    break;
				case SDL_CONTROLLER_BUTTON_BACK:
					break;
				case SDL_CONTROLLER_BUTTON_RIGHTSTICK:
					dir *= -1;
                default:
                    break;
			}
		}
		else if (sdl_event.type == SDL_CONTROLLERAXISMOTION)
        {
			float pwm_to_joystick_zone = JOYSTICK_MAXIMUM_ZONE / max_pwm;
			switch (sdl_event.caxis.axis)
			{
			case SDL_CONTROLLER_AXIS_LEFTX:
				if (abs(sdl_event.caxis.value) > abs(JOYSTICK_DEAD_ZONE))
					xAnalog_left = sdl_event.caxis.value / pwm_to_joystick_zone;
				else
					xAnalog_left = 0;
				break;
			case SDL_CONTROLLER_AXIS_LEFTY:
				if (abs(sdl_event.caxis.value) > abs(JOYSTICK_DEAD_ZONE))
					yAnalog_left = -(sdl_event.caxis.value / pwm_to_joystick_zone);
				else
					yAnalog_left = 0;
				break;
			case SDL_CONTROLLER_AXIS_TRIGGERRIGHT:
				gamepad_command = "LowerFrontFlippers";
				lastAxis = SDL_CONTROLLER_AXIS_TRIGGERRIGHT;
				break;
			case SDL_CONTROLLER_AXIS_TRIGGERLEFT:
				gamepad_command = "LowerBackFlippers";
				lastAxis = SDL_CONTROLLER_AXIS_TRIGGERLEFT;
				break;
			}
        }
	}
}


void UpdateGamepadInput()
{	
	UpdateRawInput();
	float magnitude = sqrt(xAnalog_left * xAnalog_left + yAnalog_left * yAnalog_left);
	int pwm = static_cast<int>(magnitude);
	if (pwm > max_pwm)
		pwm = max_pwm;
	else if (pwm < min_pwm)
		pwm = min_pwm;
	double theta = atan2(yAnalog_left, xAnalog_left);
	if (theta < 0)
		theta += 2 * M_PI;
	if (!isPressed && lastAxis == SDL_CONTROLLER_AXIS_INVALID)
		{
		if (xAnalog_left == 0 && yAnalog_left == 0){
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
			gamepad_value_1 *= dir;
			gamepad_value_2 *= dir;
		}
	}
}