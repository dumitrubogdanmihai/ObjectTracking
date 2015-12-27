// Object Tracker - OOP Project
// opencv 2.4.11

#include"stdafx.h"

#include<string.h>

#include "ObjectTracker.h"
#include "Helper.h" // crop object

using std::cout;

bool clicked = false;

// to select the object using the mouse
void MouseCallBack(int event, int x, int y, int flags, void* userdata);

// main window where video capture are played 
//	an object can be selected from this window simply by dragging his contour by mouse
char *mainWindow("Window");

// where each frame from video capture are stored
Mat frame;

vector<ObjectTracker> trk;

int main(int argc, char** argv)
{
	unsigned int frameCnt = 0;

	namedWindow(mainWindow);
	setMouseCallback(mainWindow, MouseCallBack, NULL);

	//computes the following path for source videos : __FILE__/../Captures/capFileName
	String capPath = __FILE__;
	String capFileName = "vid4.AVI";
	capPath = capPath.substr(0, capPath.length() - String("\\ObjectTracking\\Source.cpp").length()) + "\\Captures\\" + capFileName;
	cout << "Capture : " << "\"" + capPath + "\"" << endl << endl;
		
	VideoCapture cap(capPath); 	//Source video
	
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
			
			for_each(trk.begin(), trk.end(),
				[](ObjectTracker& tracker)
				{
					tracker.insertFrame(frame);
					tracker.highlightObject(frame, true, true, true);
					tracker.showExtraInfo();
				}
			);

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
		cout << "Beginning process of selection for object number " << trk.size() + 1 << endl;
		cout << "   Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
		
		clicked = true;
		
		initialFrame = frame.clone();
	}
	else if (event == EVENT_LBUTTONUP)
	{
		if (clicked == false)
			return;
		
		cout << "   Left button of the mouse is unclicked - position (" << x << ", " << y << ")" << endl << endl;
		
		Mat objectMat;
		char objectName[12] = "Object ";
		char cntStr[3];

		clicked = false;
		
		_itoa_s(trk.size() + 1, cntStr, 3, 10);
		strcat_s(objectName, 12, cntStr);
				
		objectMat = cropSelectedObject(initialFrame, contours[0]);
				

		Object obj(objectMat, objectName);
		obj.print();

		trk.push_back(ObjectTracker(obj));
		
		contours[0].clear();
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
