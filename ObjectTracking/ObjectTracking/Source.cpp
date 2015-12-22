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


using namespace cv;
using namespace std;

bool clicked = false;
void MouseCallBack(int event, int x, int y, int flags, void* userdata);

char *mainWindow("Window");

Mat frame;
Mat object;
ObjectTracker trk;

int main(int argc, char** argv)
{
	namedWindow(mainWindow);
	setMouseCallback(mainWindow, MouseCallBack, NULL);

	//__FILE__/../Captures
	String capFileName = "vid4.AVI";
	String capPath = __FILE__;
	capPath = capPath.substr(0, capPath.length() - String("\\ObjectTracking\\Source.cpp").length()) + "\\Captures\\" + capFileName;
	cout << "Capture : " << "\"" + capPath + "\"" << endl << endl;
	//capPath = "C:\\Users\\BOGDAN\\Desktop\\DCIM\\103NIKON\\DSCN1026.AVI";
	
	VideoCapture cap(0); 	//Source video
	
	if (cap.isOpened() == false)
	{
		cout << "Capture missing!" << endl;
		return 1;
	}


	cap.read(frame);

	//object = frame(Rect(Point(118, 73), Point(191, 170))).clone(); // skipping manual object selection by mouse
	//	imshow(searchedObjectWindow, object);
	//	waitKey(30);


		// skipping first x trivial frames
		//for (int j = 0; j < 140; j++)
			//cap.read(frame);

	while (1)
	{
		// if the selection process is not in progress
		if (!clicked)
		{
			cap.read(frame);
			if (!frame.data)
			{
				return 0;
			}
		}
		if (object.data) // if exists an object designed to be seached
		{
			trk.insertFrame(frame);
			trk.highlightObject(frame, true, true, true);
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
		if (object.data)
		{
			return;
		}

		clicked = true;
		cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
		initialFrame = frame.clone();
	}
	else if (event == EVENT_LBUTTONUP)
	{
		if (clicked == false)
			return;

		clicked = false;
		cout << "Left button of the mouse is unclicked - position (" << x << ", " << y << ")" << endl << endl;
		object = cropSelectedObject(initialFrame, contours[0], true);
				
		imshow("Object", object);
		
		trk = ObjectTracker(object);

		waitKey(30);
	}
	else if (clicked && event == EVENT_MOUSEMOVE)
	{
		if (contours[0].size() == 0)
		{
			contours[0].push_back(Point(x, y));
			circle(object, Point(x, y), 2, Scalar(255, 0, 0), 4, 8);
			cout << "Mouse move over the window - position (" << x << ", " << y << ")" << endl;
		}
		else
		{
			if (contours[0][contours[0].size() - 1].x != x || contours[0][contours[0].size() - 1].y != y)
			{
				line(frame, contours[0][contours[0].size() - 1], Point(x, y), Scalar(255, 0, 0), 3);
				contours[0].push_back(Point(x, y));
				
				circle(object, Point(x, y), 2, Scalar(255, 0, 0), 4, 8);
				cout << "Mouse move over the window - position (" << x << ", " << y << ")" << endl;
			}
		}
	}
}
