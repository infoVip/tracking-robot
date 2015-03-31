//============================================================================
// Name        : magic-mirror2.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C, Ansi-style
//============================================================================

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <ctime>

using namespace std;

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/contrib/contrib.hpp"
#include <raspicam/raspicam_cv.h>

using namespace cv;
using namespace raspicam;

const string MAIN_WINDOW_NAME = "Robot View";
const string ROI_WINDOW_NAME = "Region of Interest";
const int MAX_BINARY_VALUE = 255;

bool headless = false;

int main(int argc, const char *argv[]) {
	if (argc < 2) {
		cout << "usage: tracking-robot <threshold> [headless]" << endl;
		exit (EXIT_FAILURE);
	}
	int threshold_value = atoi(argv[1]);
	if (argc > 2) {
		cout << "Running in headless mode." << endl;
		headless = true;
	}

	if (!headless) {
		namedWindow(MAIN_WINDOW_NAME, WINDOW_AUTOSIZE);
		namedWindow(ROI_WINDOW_NAME, WINDOW_AUTOSIZE);
	}

	cout << "Opening Camera..." << endl;
	RaspiCam_Cv camera;
	//camera.set(CV_CAP_PROP_FORMAT, CV_8UC1);

	if (camera.open()) {
		Mat captured;
		int hight, width;
		int centerX, centerY;
		vector < vector <Point> > contours;
		vector<Vec4i> hierarchy;
		while (1) {
			camera.grab();
			camera.retrieve(captured);

			hight = captured.rows;
			width = captured.cols;

			centerX = width / 2;
			centerY = hight / 2;

			//circle(captured, Point(centerX, centerY), 4, Scalar(0, 255, 0));
			circle(captured, Point(centerX, centerY), 5, Scalar(0, 255, 0), -1, 8, 0);

			Rect roi(0, centerY - 45, width, 100);
			Mat roiImg = captured(roi).clone();
			cvtColor(roiImg, roiImg, COLOR_BGR2GRAY);

			threshold(roiImg, roiImg, threshold_value, MAX_BINARY_VALUE, 1);
			Mat erodeElmt = getStructuringElement(MORPH_RECT, Size(3, 3));
			Mat dilateElmt = getStructuringElement(MORPH_RECT, Size(5, 5));
			erode(roiImg, roiImg, erodeElmt);
			dilate(roiImg, roiImg, dilateElmt);

			findContours(roiImg, contours, hierarchy, CV_RETR_TREE,
					CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
			for (size_t i = 0; i < contours.size(); i++) {
				float area = contourArea(contours[i]);
				if (area > 2000) {
					Moments mu;
					mu = moments(contours[i], false);
					Point2f center(mu.m10 / mu.m00, 240); // point in center (x only)
					//circle(captured, center, 5, Scalar(0, 255, 0), -1, 8, 0);
					circle(captured, center, 5, Scalar(0, 255, 0), -1, 8, 0);
					line(captured, Point(centerX, centerY), center, Scalar(0, 255, 0));

					cout << "ak=" << centerY - 240 << ", gk=" << centerX - mu.m10 / mu.m00 << endl;
					float ankathete = centerY - 240;
					float gegenkathete = centerX - center.x;
					float bogenmass = atan(gegenkathete/ankathete);
					float gradmass = bogenmass*180/3.1415926535;
					cout << "winkel=" << gradmass << " Grad" << endl;
					//putText(captured, text, textOrg, fontFace, fontScale,
					//        Scalar::all(255), thickness, 8);
				}
			}

			if (!headless) {
				imshow(MAIN_WINDOW_NAME, captured);
				imshow(ROI_WINDOW_NAME, roiImg);
				waitKey(30);
			}
		}
	} else {
		cout << "failed to open camera." << endl;
		exit (EXIT_FAILURE);
	}

	return EXIT_SUCCESS;
}
