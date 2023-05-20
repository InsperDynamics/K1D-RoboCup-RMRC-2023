#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <numeric>
#include <fstream>
using namespace std;
using namespace cv;

static Mat image;
static ofstream myfile;
QRCodeDetector qrDetector;
vector<Point> points;
string qrContent;

static Point getCentroid(InputArray Points)
{
	Point Coord;
	Moments mm = moments(Points, false);
	double moment10 = mm.m10;
	double moment01 = mm.m01;
	double moment00 = mm.m00;
	Coord.x = int(moment10 / moment00);
	Coord.y = int(moment01 / moment00);
	return Coord;
}

static void InitializeQR() 
{
	myfile.open("qr_results.txt");
}

static Mat ReadQR(Mat image)
{
	Mat frame;
	image.copyTo(frame);
    qrContent = qrDetector.detectAndDecode(frame, points);
	if (!qrContent.empty()) 
	{
		if (points.size() == 4)
		{
			line(frame, points[0], points[1], Scalar(0, 255, 0), 2);
			line(frame, points[1], points[2], Scalar(0, 255, 0), 2);
			line(frame, points[2], points[3], Scalar(0, 255, 0), 2);
			line(frame, points[3], points[0], Scalar(0, 255, 0), 2);
		}
		putText(frame, qrContent, getCentroid(points), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 2);
		myfile << qrContent << endl;
	}
	return image;
}