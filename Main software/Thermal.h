#pragma once
#include <iostream>
#include <string>
#include "opencv2/opencv.hpp"
using namespace std;
using namespace cv;
#define MAX_TEMP_VALUE 30
#define MIN_TEMP_VALUE 20
const int thermal_width = 8, thermal_height = 8, upscale_factor = 20;
const int thermal_width_upscaled = thermal_width * upscale_factor;
const int thermal_height_upscaled = thermal_height * upscale_factor;
Mat thermal_image = Mat::zeros(Size(thermal_width, thermal_height), CV_8UC3);

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
	resize(thermal_image, thermal_image, Size(), upscale_factor, upscale_factor, INTER_CUBIC);
}