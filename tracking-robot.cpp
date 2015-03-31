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
const int LOOK_AHEAD = 240;

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

	RaspiCam_Cv camera;

	if (camera.open()) {
		Mat captured;
		vector < vector <Point> > contours;
		vector<Vec4i> hierarchy;
		while (1) {
			camera.grab();
			camera.retrieve(captured);

			Point capturedImageCenter(captured.cols / 2, captured.rows / 2);
			circle(captured, capturedImageCenter, 5, Scalar(0, 255, 0), -1, 8, 0);

			// region of interest. currently around the center of the
			// captured image. might need to be moved depending on the
			// camera angle, speed of the robot etc.
			Rect roi(0, capturedImageCenter.y - 45, captured.cols, 100);
			Mat roiImg = captured(roi).clone();

			// extract the line to follow from the background
			cvtColor(roiImg, roiImg, COLOR_BGR2GRAY);
			threshold(roiImg, roiImg, threshold_value, MAX_BINARY_VALUE, 1);
			Mat erodeElmt = getStructuringElement(MORPH_RECT, Size(3, 3));
			Mat dilateElmt = getStructuringElement(MORPH_RECT, Size(5, 5));
			erode(roiImg, roiImg, erodeElmt);
			dilate(roiImg, roiImg, dilateElmt);

			// determine the angle the robot needs to move
			findContours(roiImg, contours, hierarchy, CV_RETR_TREE,
					CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
			for (size_t i = 0; i < contours.size(); i++) {
				float area = contourArea(contours[i]);
				if (area > 2000) {
					Moments mu;
					mu = moments(contours[i], false);
					Point2f roiCenter(mu.m10 / mu.m00, capturedImageCenter.y - LOOK_AHEAD); // point in center (x only)
					circle(captured, roiCenter, 5, Scalar(0, 255, 0), -1, 8, 0);
					line(captured, capturedImageCenter, roiCenter, Scalar(0, 255, 0));

					float ankathete = LOOK_AHEAD;
					float gegenkathete = capturedImageCenter.x - roiCenter.x;
					float bogenmass = atan(gegenkathete/ankathete);
					float gradmass = bogenmass*180/3.1415926535;

					string caption = format("%f Grad", gradmass);
					putText(captured, caption, Point(capturedImageCenter.x + 20, capturedImageCenter.y + 20),  FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 0));
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
