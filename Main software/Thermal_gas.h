#pragma once
#include <iostream>
#include <string>
#include "opencv2/opencv.hpp"
using namespace std;
using namespace cv;
#define MAX_GAS_VALUE 3000
#define MAX_TEMP_VALUE 30
#define MIN_TEMP_VALUE 20
const int thermal_width = 8, thermal_height = 8, upscale_factor = 20;
const int thermal_width_upscaled = thermal_width * upscale_factor;
const int thermal_height_upscaled = thermal_height * upscale_factor;
Mat thermal_image = Mat::zeros(Size(thermal_width, thermal_height), CV_8UC3);
Mat gas_image = Mat::zeros(Size(thermal_width_upscaled, thermal_height_upscaled * 0.4), CV_8UC3);

void UpdateGas(int current_gas)
{
	gas_image = Mat::zeros(Size(thermal_width_upscaled, thermal_height_upscaled * 0.4), CV_8UC3);
	float rectangle_width = (float(current_gas) / MAX_GAS_VALUE) * float(thermal_width_upscaled);
	rectangle(gas_image, Point(0, 0), Point(thermal_width_upscaled, thermal_height_upscaled * 0.4), Scalar(255, 255, 255), FILLED);
	rectangle(gas_image, Point(0, 0), Point(rectangle_width, thermal_height_upscaled * 0.4), Scalar(0, 255, 255), FILLED);
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