#include"stdafx.h"
#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/imgproc/imgproc.hpp" // convex Hull

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

	String capFileName = "vid4.AVI";
	String capPath = __FILE__;
	capPath = capPath.substr(0, capPath.length() - String("\\ObjectTracking\\Source.cpp").length()) + "\\Captures\\" + capFileName;
	cout << "Capture : " << "\"" + capPath + "\"" << endl << endl;
	
	VideoCapture cap(capPath); // source
	if (cap.isOpened() == false)
	{
		cout << "Capture missing!" << endl;
		return 1;
	}

	namedWindow(mainWindow);
	setMouseCallback(mainWindow, MouseCallBack, NULL);
	
	//VideoCapture cap("vid4.AVI"); // source

	vector<Point> contour;
	vector<vector<Point>> cntr(1); // controur converted for printing  function

	Point p1, p2; // corners of the finded object  (NV & SE)
	Point pMin, pMax; // corners of the area where the object is searched  (NV & SE)

	Scalar contourColor;

	cap.read(frame);
	object = frame(Rect(Point(118, 73), Point(191, 170))).clone();
	//object = frame(Rect(Point(18, 73), Point(91, 170))).clone();
		imshow(searchedObjectWindow, object);
		waitKey(30);

	Rect boundedRect;
	Mat boundedObject(object.clone());
	
		// skip first 140 frames
		//for (int j = 0; j < 140; j++)
			cap.read(frame);

	while (1)
	{
		if (!clicked)
		{
			cap.read(frame);
			cout << "Frmae : " << ++frameCnt << "  ";

			if (!frame.data)
			{
				return 0;
			}
		}
		if (object.data)
		{
			found ? kalmanFilter.predict(predP, predR) : 0;

			contourColor = Scalar(0, 255, 0);

			boundedRect = boundPoints(10, p1, p2, pMin, pMax, frame); // expand last contour
			
			boundedObject = Mat(frame(boundedRect).clone()); // search just in small area of the whole frame ( boundedObject)
			
			contour = findObj(object, boundedObject); // TRY to : find the object and extract his contour
						
			if (objectFinded(contour, frame, pMin) == false || contour.size() < 4) // if the object wasn't find/ it's contour is wrong
			{
				kalmanFilter.update();

				p1 = Point(0, 0);							// expand the search to the whole frame
				p2 = Point(frame.cols, frame.rows);

				contourColor = Scalar(0, 0, 240);			// outline the contour with other colour

				//object = lastObjectFrame.clone(); actualizare obiect cautat

				imshow(searchedObjectWindow, object); waitKey(30);
			}
			else // find p1 and p2 IF the object was finded
			{
				minNormalRect(contour, p1, p2); // find the corners of contour

				p1 += pMin; //offset p1 & p2 
				p2 += pMin;

				found = true;
				kalmanFilter.update(Point((p1.x + p2.x) / 2, (p1.y + p2.y) / 2), Rect(p1, p2));
			}
			cntr[0] = contour;
		
			//offset contour
			for (int j = 0; j < cntr[0].size(); j++)
			{
				cntr[0][j].x += pMin.x;
				cntr[0][j].y += pMin.y;
			}

			// if object was finded
			if (contourColor != Scalar(0, 0, 240))
			{
				lastObjectFrame = cropSelectedObject(frame, cntr[0]).clone(); // save it 
				imshow(findedObjectWindow, lastObjectFrame); waitKey(30);
			}
			else
			{
				putText(frame, "Predicted", Point(predR.x, predR.y+5), FONT_HERSHEY_COMPLEX_SMALL, 0.5, Scalar(150, 150, 150), 1, CV_AA);
				rectangle(frame, predR, Scalar(150, 150, 150), 2); // if the ofject wasn't finded print it's predicted rectangle
				circle(frame, predP, 5, Scalar(150, 150, 150), 2);
			}
			drawDetails(frame, cntr, contourColor, boundedRect, p1, p2, pMin, pMax); // draw his contour and other userfull infos
			kalmanFilter.printTrajectories(frame.size());

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



void MouseCallBack(int event, int x, int y, int flags, void* userdata)
{
	static vector<vector<Point>> contours(1);
	static Mat initialFrame;

	if (event == EVENT_LBUTTONDOWN)
	{
		clicked = true;
		cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
		initialFrame = frame.clone();
	}
	else if (event == EVENT_LBUTTONUP)
	{
		clicked = false;
		cout << "Left button of the mouse is unclicked - position (" << x << ", " << y << ")" << endl << endl;
		object = cropSelectedObject(initialFrame, contours[0]);
				
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
