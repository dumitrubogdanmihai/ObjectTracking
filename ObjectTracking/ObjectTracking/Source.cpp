#include"stdafx.h"
#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/imgproc/imgproc.hpp" // convex Hull

#include "GraphUtils.h"
#include "MovementAnalysis.h"

#include "Helper.h"
#include "Finder.h"
#include "Kalman.h"
#include "ObjectTracker.h"
#include "Object.h"


using namespace cv;
using namespace std;

bool clicked = false;
void MouseCallBack(int event, int x, int y, int flags, void* userdata);

char *mainWindow("Window");

Mat frame;

int trkCnt = 0;

ObjectTracker trk[10];

int main(int argc, char** argv)
{
	unsigned int frameCnt = 0;

	namedWindow(mainWindow);
	setMouseCallback(mainWindow, MouseCallBack, NULL);



	//computes the following path for source videos : __FILE__/../Captures
	String capFileName = "vid4.AVI";
	String capPath = __FILE__;
	capPath = capPath.substr(0, capPath.length() - String("\\ObjectTracking\\Source.cpp").length()) + "\\Captures\\" + capFileName;
	cout << "Capture : " << "\"" + capPath + "\"" << endl << endl;
		
	VideoCapture cap(0); 	//Source video
	
	if (cap.isOpened() == false)
	{
		cout << "Capture missing!" << endl;
		return 1;
	}
	
	frameCnt++;
	cap.read(frame);

	while (1)
	{
		// if the selection process is not in progress
		if (!clicked)
		{
			frameCnt++;
			cap.read(frame);
			if (!frame.data)
			{
				return 0;
			}

			cout << endl  << "FRAME " << frameCnt << endl;

			for (int i = 0; i < trkCnt; i++)
			{
				trk[i].insertFrame(frame);
				trk[i].highlightObject(frame, true, true, true);
			}
		}

		imshow(mainWindow, frame);

		if (waitKey(30) == 27)
			break;
	}
}


void MouseCallBack(int event, int x, int y, int flags, void* userdata) // designed for manual selecting of object contour
{
	static vector<vector<Point>> contours(1);
	static Mat initialFrame;

	if (event == EVENT_LBUTTONDOWN)
	{
		if (trkCnt > 8)
		{
			return;
		}

		clicked = true;
		cout << "Beginning process of selection for object number " << trkCnt + 1 << endl;
		cout << "   Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
		initialFrame = frame.clone();
	}
	else if (event == EVENT_LBUTTONUP)
	{
		if (clicked == false)
			return;
		
		char cntStr[3];
		_itoa_s(trkCnt + 1, cntStr, 3, 10);
		String objectWindowName = "Object ";
		objectWindowName += cntStr;
		
		Mat object;
		
		clicked = false;
		cout << "   Left button of the mouse is unclicked - position (" << x << ", " << y << ")" << endl << endl;
		
		object = cropSelectedObject(initialFrame, contours[0], true);
		imshow(objectWindowName, object);
		
		contours[0].clear();

		Object obj(object);
		trk[trkCnt] = ObjectTracker(obj);
		
		trkCnt++;

		waitKey(30);
	}
	else if (clicked && event == EVENT_MOUSEMOVE)
	{
		if (contours[0].size() != 0)
		{
			line(frame, contours[0].back(), Point(x, y), Scalar(255, 0, 0), 3);
		}
		if (contours[0].size() == 0 || (contours[0][contours[0].size() - 1].x != x || contours[0][contours[0].size() - 1].y != y))
		{
			cout << "   Mouse move over the window - position (" << x << ", " << y << ")" << endl;
			contours[0].push_back(Point(x, y));
		}
	}
}
