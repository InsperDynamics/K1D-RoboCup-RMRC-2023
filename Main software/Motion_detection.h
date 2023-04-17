#pragma once
#include "opencv2/opencv.hpp"
#include <iostream>
using namespace std;
using namespace cv;

Mat frame1, frame2, hsv;

const double PI = 3.14159265358979323846;

static Mat DetectMotion(Mat image)
{
	frame2 = image;
	if (frame1.empty())
	{
		frame1 = frame2;
		return frame2;
	}
	GaussianBlur(frame2, frame2, Size(5, 5), 0);
	cvtColor(frame2, frame2, COLOR_BGR2GRAY);
	Mat flow(frame1.size(), CV_32FC2);
	calcOpticalFlowFarneback(frame1, frame2, flow, 0.5, 3, 15, 3, 5, 1.2, 0);
	Mat magnitude, angle;
	cartToPolar(flow.col(0), flow.col(1), magnitude, angle, true);
	hsv.setTo(Scalar(0, 0, 0)); // set H and V channels to 0
	angle *= 180 / PI / 2;
	hsv(Range::all(), Range::all(), 0) = angle;
	normalize(magnitude, magnitude, 0, 255, NORM_MINMAX);
	threshold(magnitude, magnitude, 50, 255, THRESH_TOZERO);
	hsv(Range::all(), Range::all(), 2) = magnitude;
	Mat bgr;
	cvtColor(hsv, bgr, COLOR_HSV2BGR);
	frame1 = frame2;
	return bgr;
}