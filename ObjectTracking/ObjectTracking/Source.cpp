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
#include "MovementAnalisys.h"

#include "Helper.h"
#include "Finder.h"
#include "Kalman.h"


using namespace cv;
using namespace std;

void MouseCallBack(int event, int x, int y, int flags, void* userdata);

Mat object;
Mat frame;
Mat lastObjectFrame;
bool clicked = false;
char *mainWindow("Window");
char *searchedObjectWindow("Searched Object");
char *findedObjectWindow("Finded Object");

bool found = false; // if the object was founded at least one time 


int main(int argc, char** argv)
{
	unsigned int frameCnt = 0;
	Point predP;
	Rect predR;

	CKalmanFilter kalmanFilter = CKalmanFilter();

	//__FILE__/../Captures
	String capFileName = "vid1.AVI";
	String capPath = __FILE__;
	capPath = capPath.substr(0, capPath.length() - String("\\ObjectTracking\\Source.cpp").length()) + "\\Captures\\" + capFileName;
	cout << "Capture : " << "\"" + capPath + "\"" << endl << endl;
	//capPath = "C:\\Users\\BOGDAN\\Desktop\\DCIM\\103NIKON\\DSCN1026.AVI";
	//Source video
	VideoCapture cap(capPath); 
	if (cap.isOpened() == false)
	{
		cout << "Capture missing!" << endl;
		return 1;
	}

	//for (int j = 0; j < 63; j++)
	//	cap.read(frame);
	//object = frame(Rect(Point(132, 175), Point(370, 413))).clone(); // skipping manual object selection by mouse

	namedWindow(mainWindow);
	setMouseCallback(mainWindow, MouseCallBack, NULL);
	
	vector<Point> contour;
	vector<vector<Point>> cntr(1); // controur converted for printing  function

	Point p1, p2; // corners of the finded object  (NV & SE)
	Point pMin, pMax; // corners of the area where the object is searched  (NV & SE)
	Scalar contourColor;

	cap.read(frame);
	//object = frame(Rect(Point(118, 73), Point(191, 170))).clone(); // skipping manual object selection by mouse
	//	imshow(searchedObjectWindow, object);
	//	waitKey(30);

	Rect boundedRect;
	Mat boundedObject(object.clone()); // limited area for efficient object searching 
	
		// skipping first x trivial frames
		//for (int j = 0; j < 140; j++)
			cap.read(frame);
	// main loop
			MovementAnalisys ma;
	while (1)
	{
		Mat frClone;
		// if the selection process is not in progress
		if (!clicked)
		{
			cap.read(frame);
			frClone = frame.clone();
			cout << endl << "   Frmae " << ++frameCnt << endl;
			if (!frame.data)
			{
				return 0;
			}
		}
		if (object.data) // if exists an object designed to be seached
		{
			found ? kalmanFilter.predict(predP, predR) : 0; // if the obj was finded in the past predict it's tarjectory

			contourColor = Scalar(0, 255, 0); // set an contour color

			boundedRect = boundPoints(10, p1, p2, pMin, pMax, frame); // expand last contour
			
			boundedObject = Mat(frame(boundedRect).clone()); // search just in small area of the whole frame ( boundedObject)
			
			contour = findObj(object, boundedObject); // TRY to : find the object and extract his contour
						
			if (objectFinded(contour, frame, pMin) == false || contour.size() < 4) // if the object wasn't finded/ it's contour is wrong
			{
				ma.insert();
				kalmanFilter.update(); // update kalman without object's new coordinates

				// expand the search to the whole frame
				p1 = Point(0, 0);
				p2 = Point(frame.cols, frame.rows);

				contourColor = Scalar(0, 0, 240);// outline the contour with other colour

				/*
				update the searched object frame
				userfull in case of complet object trasition when the initial frame doesn't corespond with current object's frame
				*/
				//object = lastObjectFrame.clone(); 

				imshow(searchedObjectWindow, object); waitKey(30);
			}

			// if the object was finded
			else // compute p1 and p2 from object's contour
			{
				minNormalRect(contour, p1, p2); // find the corners of contour

				p1 += pMin; //offset p1 & p2 
				p2 += pMin;

				found = true;
				ma.insert(true, Point((p1.x + p2.x) / 2, (p1.y + p2.y) / 2), Rect(p1, p2)); // update with new data
				kalmanFilter.update(Point((p1.x + p2.x) / 2, (p1.y + p2.y) / 2), Rect(p1, p2)); // update with new data
			}
			cntr[0] = contour;//designated for contour printing (with OpenCV build-in "drawContours()" function)
		
			//offset contour
			for (int j = 0; j < cntr[0].size(); j++)
			{
				cntr[0][j].x += pMin.x;
				cntr[0][j].y += pMin.y;
			}

			// if object was finded
			if (contourColor != Scalar(0, 0, 240))
			{
				lastObjectFrame = cropSelectedObject(frame, cntr[0]).clone(); // save the new objec frame
				imshow(findedObjectWindow, lastObjectFrame); waitKey(30);
			}
			else// if object wasn't finded 
			{
				//print it's predicted trajectory by using kalman filter
				putText(frame, "Predicted", Point(predR.x, predR.y+5), FONT_HERSHEY_COMPLEX_SMALL, 0.5, Scalar(150, 150, 150), 1, CV_AA);
				rectangle(frame, predR, Scalar(150, 150, 150), 2); // if the ofject wasn't finded print it's predicted rectangle
				circle(frame, predP, 5, Scalar(150, 150, 150), 2);
			}
			drawDetails(frame, cntr, contourColor, boundedRect, p1, p2, pMin, pMax); // draw his contour and other userfull infos
			
			// print in a separate window object's trajectory
			kalmanFilter.printTrajectories(frame.size());
			ma.printAnalisys(frClone);


			imshow(mainWindow, frame); waitKey(30);

		}
		else
		{
			imshow(mainWindow, frame);
		}

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
