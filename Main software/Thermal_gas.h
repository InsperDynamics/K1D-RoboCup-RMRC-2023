#pragma once
#include <iostream>
#include <string>
#include "opencv2/opencv.hpp"
using namespace std;
using namespace cv;
#define MAX_GAS_VALUE 3000
#define MAX_TEMP_VALUE 35
#define MIN_TEMP_VALUE 15
const int thermal_width = 32, thermal_height = 24, upscale_factor = 10;
const int thermal_width_upscaled = thermal_width * upscale_factor;
const int thermal_height_upscaled = thermal_height * upscale_factor;
const int gas_width = 160, gas_height = 64;
Mat thermal_image = Mat::zeros(Size(thermal_width, thermal_height), CV_8UC3);
Mat gas_image = Mat::zeros(Size(gas_width, gas_height), CV_8UC3);

void UpdateGas(int current_gas)
{
	gas_image = Mat::zeros(Size(gas_width, gas_height), CV_8UC3);
	float rectangle_width = (float(current_gas) / MAX_GAS_VALUE) * float(gas_width);
	rectangle(gas_image, Point(0, 0), Point(gas_width, gas_height), Scalar(255, 255, 255), FILLED);
	rectangle(gas_image, Point(0, 0), Point(rectangle_width, gas_height), Scalar(0, 255, 255), FILLED);
	putText(gas_image, to_string(current_gas) + " ppm", Point(0, gas_image.rows / 2), FONT_HERSHEY_DUPLEX, 1, Scalar(0, 0, 0));
}

void UpdateThermal(float current_temperature[thermal_width * thermal_height])
{
	thermal_image = Mat::zeros(Size(thermal_width, thermal_height), CV_8UC3);
	for (int x=0 ; x<thermal_width ; x++)
	{
		for (int y=0 ; y<thermal_height ; y++)
		{
			int temp = int(current_temperature[x + y * thermal_width]);
			Vec3b color = thermal_image.at<Vec3b>(Point(x,y));
			color[0] = int(255 / (MAX_TEMP_VALUE - MIN_TEMP_VALUE) * (MAX_TEMP_VALUE - temp));
			color[1] = 0;
			color[2] = int(255 / (MAX_TEMP_VALUE - MIN_TEMP_VALUE) * (temp - MIN_TEMP_VALUE));
			thermal_image.at<Vec3b>(Point(x,y)) = color;
		}
	}
	transpose(thermal_image, thermal_image);  
	flip(thermal_image, thermal_image ,1);
	resize(thermal_image, thermal_image, Size(), upscale_factor, upscale_factor, INTER_CUBIC);
}