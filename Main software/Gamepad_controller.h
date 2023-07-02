#pragma once
#include <iostream>
#include <math.h>
#include <SDL2/SDL.h>
#define JOYSTICK_DEAD_ZONE 4000
#define JOYSTICK_MAXIMUM_ZONE 33000
#define DELTA 30
#define FLIPPER_DELTA 100
#define GRIPPER_DELTA 45
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
int macro = 0;
int macro_flipper = 0;
int autonomous_mode = 0;
int dexterity_mode = 0;
int qr_detection = 0;
int hazmat_detection = 0;
int motion_detection = 0;

int dir = 1;

void InitializeGamepad()
{
	SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMECONTROLLER);
    gGameController = SDL_GameControllerOpen(0);
}

void UpdateRawInput()
{	
	while(SDL_PollEvent(&sdl_event))
	{
		if (sdl_event.cbutton.type == SDL_CONTROLLERBUTTONUP)
		{
			isPressed = false;
			if (sdl_event.cbutton.button == SDL_CONTROLLER_BUTTON_BACK) {
				macro = !macro;
				while(SDL_PollEvent(&sdl_event)){}
			}
			else if (sdl_event.cbutton.button == SDL_CONTROLLER_BUTTON_START) {
				macro_flipper = !macro_flipper;
				while(SDL_PollEvent(&sdl_event)){}
			}
			else if (sdl_event.cbutton.button == SDL_CONTROLLER_BUTTON_X  && macro) {
				motion_detection = !motion_detection;
				while(SDL_PollEvent(&sdl_event)){}
			}
			else if (sdl_event.cbutton.button == SDL_CONTROLLER_BUTTON_A && macro) {
				qr_detection = !qr_detection;
				while(SDL_PollEvent(&sdl_event)){}
			}
			else if (sdl_event.cbutton.button == SDL_CONTROLLER_BUTTON_B && macro) {
				hazmat_detection = !hazmat_detection;
				while(SDL_PollEvent(&sdl_event)){}
			}
			else if (sdl_event.cbutton.button == SDL_CONTROLLER_BUTTON_Y && macro) {
				temperature_gas_mode = !temperature_gas_mode;
				while(SDL_PollEvent(&sdl_event)){}
			}
		} 

		if (SDL_GameControllerGetAxis(gGameController, lastAxis) < JOYSTICK_DEAD_ZONE)
		{
			lastAxis = SDL_CONTROLLER_AXIS_INVALID;
		}

		if (sdl_event.cbutton.type == SDL_CONTROLLERBUTTONDOWN)
		{
			isPressed = true;

			switch (sdl_event.cbutton.button)
			{
				case SDL_CONTROLLER_BUTTON_RIGHTSHOULDER:
					if (macro_flipper) {
						gamepad_command = "LowerIndividualFlipper";
					}
					else {
						gamepad_command = "RaiseIndividualFlipper";
					}
					gamepad_value_1 = 1;
					gamepad_value_2 = FLIPPER_DELTA;
					if (SDL_GameControllerGetButton(gGameController, SDL_CONTROLLER_BUTTON_LEFTSHOULDER)) {
						if (macro_flipper) {
							gamepad_command = "LowerFrontFlippers";
						}
						else {
							gamepad_command = "RaiseFrontFlippers";
						}
						gamepad_value_1 = FLIPPER_DELTA;
						while(SDL_PollEvent(&sdl_event)){}
					}
					break;
				case SDL_CONTROLLER_BUTTON_LEFTSHOULDER:
				    if (macro_flipper) {
						gamepad_command = "LowerIndividualFlipper";
					}
					else {
						gamepad_command = "RaiseIndividualFlipper";
					}
					gamepad_value_1 = 0;
					gamepad_value_2 = FLIPPER_DELTA;
					if (SDL_GameControllerGetButton(gGameController, SDL_CONTROLLER_BUTTON_RIGHTSHOULDER)) {
						if (macro_flipper) {
							gamepad_command = "LowerFrontFlippers";
						}
						else {
							gamepad_command = "RaiseFrontFlippers";
						}
						gamepad_value_1 = FLIPPER_DELTA;
						while(SDL_PollEvent(&sdl_event)){}
					}
                    break;
				case SDL_CONTROLLER_BUTTON_Y:
					if (!macro) {
						gamepad_command = "Third+";
						gamepad_value_1 = DELTA;
					}
					break;
				case SDL_CONTROLLER_BUTTON_A:
					if (!macro) {
						gamepad_command = "Third-";
						gamepad_value_1 = DELTA;
					}
                    break;
				case SDL_CONTROLLER_BUTTON_DPAD_UP:
					gamepad_command = "Second+";
					gamepad_value_1 = DELTA;
                    break;
				case SDL_CONTROLLER_BUTTON_DPAD_DOWN:
					gamepad_command = "Second-";
					gamepad_value_1 = DELTA;
                    break;
				case SDL_CONTROLLER_BUTTON_DPAD_RIGHT:
					gamepad_command = "First-";
					gamepad_value_1 = DELTA;
                    break;
				case SDL_CONTROLLER_BUTTON_DPAD_LEFT:
					gamepad_command = "First+";
					gamepad_value_1 = DELTA;
                    break;
				case SDL_CONTROLLER_BUTTON_X:
					if (!macro) {
						gamepad_command = "ClawOpen";
						gamepad_value_1 = GRIPPER_DELTA;
					}
				    break;
                case SDL_CONTROLLER_BUTTON_B:
					if (!macro){
						gamepad_command = "ClawClose";
						gamepad_value_1 = GRIPPER_DELTA;
					}
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
				if (macro_flipper) {
					gamepad_command = "LowerIndividualFlipper";
				}
				else {
					gamepad_command = "RaiseIndividualFlipper";
				}
				gamepad_value_1 = 3;
				gamepad_value_2 = FLIPPER_DELTA;
				if (SDL_GameControllerGetAxis(gGameController, SDL_CONTROLLER_AXIS_TRIGGERLEFT) >= JOYSTICK_DEAD_ZONE) {
					if (macro_flipper) {
						gamepad_command = "LowerBackFlippers";
					}
					else {
						gamepad_command = "RaiseBackFlippers";
					}
					gamepad_value_1 = FLIPPER_DELTA;
					while(SDL_PollEvent(&sdl_event)){}
				}
				lastAxis = SDL_CONTROLLER_AXIS_TRIGGERRIGHT;
				break;
			case SDL_CONTROLLER_AXIS_TRIGGERLEFT:
				if (macro_flipper) {
					gamepad_command = "LowerIndividualFlipper";
				}
				else {
					gamepad_command = "RaiseIndividualFlipper";
				}
				gamepad_value_1 = 2;
				gamepad_value_2 = FLIPPER_DELTA;
				if (SDL_GameControllerGetAxis(gGameController, SDL_CONTROLLER_AXIS_TRIGGERRIGHT) >= JOYSTICK_DEAD_ZONE) {
					if (macro_flipper) {
						gamepad_command = "LowerBackFlippers";
					}
					else {
						gamepad_command = "RaiseBackFlippers";
					}
					gamepad_value_1 = FLIPPER_DELTA;
					while(SDL_PollEvent(&sdl_event)){}
				}
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