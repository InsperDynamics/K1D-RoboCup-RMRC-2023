#pragma once
#include "opencv2/opencv.hpp"
#include <iostream>
using namespace std;
using namespace cv;

Mat firstFrame;

static Mat DetectMotionAbsdiff(Mat image)
{
	Mat blurred;
	GaussianBlur(image, blurred, Size(3, 3), 0);
	if (firstFrame.empty())
		firstFrame = blurred;
	Mat frameDelta;
	absdiff(firstFrame, blurred, frameDelta);
	cvtColor(frameDelta, frameDelta, COLOR_BGR2GRAY);
	Mat DeltaThresholded;
	double thresh = threshold(frameDelta, DeltaThresholded, 50, 255, THRESH_BINARY);
	Mat dilate_kernel = getStructuringElement(MORPH_RECT, Size(5, 5), Point(2, 2));
	dilate(DeltaThresholded, DeltaThresholded, dilate_kernel, Point(-1, -1), 2);
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	DeltaThresholded.convertTo(DeltaThresholded, CV_8UC1);
	findContours(DeltaThresholded, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
	for (size_t c = 0; c < contours.size(); c++)
	{
		if (0.5 < boundingRect(contours[c]).height / boundingRect(contours[c]).width < 2) 
		{
			drawContours(image, contours, (int)c, Scalar(0, 0, 255), 2);
		}
	}
	firstFrame = blurred;
	return image;
}

static Mat DetectMotionOpticalFlow(Mat image)
{
	Mat gray;
	cvtColor(image, gray, COLOR_BGR2GRAY);
	if (firstFrame.empty())
		firstFrame = gray;
	Mat flow(firstFrame.size(), CV_32FC2);
	optflow:calcOpticalFlowFarneback(firstFrame, gray, flow, 0.2, 10, 15, 10, 5, 1.2, 0);
	Mat flow_parts[2];
	split(flow, flow_parts);
	Mat magnitude, angle, magn_norm;
	cartToPolar(flow_parts[0], flow_parts[1], magnitude, angle, true);
	normalize(magnitude, magn_norm, 0.0f, 1.0f, NORM_MINMAX);
	angle *= ((1.1f / 360.f) * (180.f / 255.f));
	Mat _hsv[3], hsv, hsv8, bgr;
	_hsv[0] = angle;
	_hsv[1] = Mat::ones(angle.size(), CV_32F);
	_hsv[2] = magn_norm;
	merge(_hsv, 3, hsv);
	hsv.convertTo(hsv8, CV_8U, 255.0);
	cvtColor(hsv8, bgr, COLOR_HSV2BGR);
	firstFrame = gray;
	return bgr;
}