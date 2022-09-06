#pragma once
#include <zbar.h>
#include <opencv2/opencv.hpp>
#include <numeric>
#include <fstream>
using namespace std;
using namespace cv;
using namespace zbar;

static ImageScanner scanner;
static Mat image;
static Mat imageGray;
static vector<Point2f> obj_location;
static vector<string> qr_results{};
static ofstream myfile;

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
	scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 0);
	scanner.set_config(ZBAR_QRCODE, ZBAR_CFG_ENABLE, 1);
	scanner.set_config(ZBAR_QRCODE, ZBAR_CFG_X_DENSITY, 0);
	scanner.set_config(ZBAR_QRCODE, ZBAR_CFG_Y_DENSITY, 0);
	myfile.open("qr_results.txt");
}


static Mat ReadQR(Mat image)
{
	Mat imageGray;
	cvtColor(image, imageGray, COLOR_RGB2GRAY);
	int width = imageGray.cols;
	int height = imageGray.rows;
	uchar* raw = (uchar*)imageGray.data;
	Image imageZbar(width, height, "Y800", raw, width * height);
	scanner.scan(imageZbar);
	Image::SymbolIterator symbol = imageZbar.symbol_begin();
	if (imageZbar.symbol_begin() != imageZbar.symbol_end())
	{
		for (; symbol != imageZbar.symbol_end(); ++symbol)
		{
			for (int i = 0; i < symbol->get_location_size(); i++)
			{
				obj_location.push_back(Point(symbol->get_location_x(i), symbol->get_location_y(i)));
			}
			for (int i = 0; i < obj_location.size(); i++)
			{
				line(image, obj_location[i], obj_location[(i + 1) % obj_location.size()], Scalar(255, 0, 0), 3);
			}
			String text = symbol->get_data();
			int fontFace = FONT_HERSHEY_SIMPLEX;
			double fontScale = 1.1;
			int thickness = 2;
			Size textSize = getTextSize(text, fontFace, fontScale, thickness, 0);
			Point textOrg(getCentroid(obj_location).x - (textSize.width / 2), getCentroid(obj_location).y);
			putText(image, text, textOrg, fontFace, fontScale, (0, 0, 255), thickness);
			obj_location.clear();
			if (find(qr_results.begin(), qr_results.end(), symbol->get_data()) == qr_results.end())
				myfile << "\nDecode Result: " << endl << symbol->get_data() << endl;
			qr_results.push_back(symbol->get_data());
		}
		imageZbar.set_data(NULL, 0);
	}
	return image;
}