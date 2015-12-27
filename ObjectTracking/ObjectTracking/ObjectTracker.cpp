#include "ObjectTracker.h"
#include "ObjectFinder.h"
#include "Object.h"

#include "Helper.h"
#include "Finder.h"

using namespace std;

ObjectTracker::ObjectTracker()
{
	foundedInLastFrame = false;
	foundedAtLeastOnce = false;
}

ObjectTracker::ObjectTracker(Object &obj)
{
	// object that have to be tracked by one instance of ObjectTracker class
	this->object = obj;

	foundedInLastFrame = false;
	foundedAtLeastOnce = false;
}

ObjectTracker::~ObjectTracker()
{
	cout << "ObjectTracker class destructor was called!" << endl;
}

void ObjectTracker::insertFrame(Mat &frame)
{


	// if object was finded in last frame searchingArea is limited
	//  to a ROI from frame by bolding object's rectangle from last frame 
	// else searchedArea represents whole frame 
	Mat searchedArea;

	// represents edges of searchedArea ROI 
	//	NV point = pMin 
	//	SE point = pMax
	// OBS: pMin stand-up for decalation from original frame 
	//	and it is used to offset contour resulted from ROI to original frame
	Point pMin;
	Point pMax;

	// predicted center and rectangle
	Point predC;
	Rect predR;

	foundedAtLeastOnce ? MA.getPred(predC, predR) : 0;

	// set searchedArea
	if (foundedInLastFrame)
	{
		// small area from frame where the object should be
		//	based by object position in last frame
		Rect boundedRect = boundPoints(p1, p2, 10, pMin, pMax, frame);
	
		// selecting ROI from frame
		searchedArea = Mat(frame(boundedRect)).clone(); 
	}
	else
	{
		// set searchedArea to whole frame
		searchedArea = frame;
	}
		
	// search object and extract contour
	finder.find(object, frame, contour);
	
	// object FINDED
	if (finder.objectFinded(contour, frame, pMin))
	{
		foundedInLastFrame = true;
		foundedAtLeastOnce == false ? foundedAtLeastOnce = true : 0;
		founded = true;
		predicted = false;

		//  offset contour to point to whole frame
		for (unsigned int j = 0; j < contour.size(); j++)
		{
			contour[j].x += pMin.x;
			contour[j].y += pMin.y;
		}

		// update last seen
		Mat objNewCap = cropSelectedObject(frame, contour).clone();
		object.insertCapture(objNewCap);

		// update object's center and it's rectangle
		getRectAndCenter(contour, center, rect);

		// insert new info to MovementAnalysis
		MA.insert(true, center, rect);
	}

	// object NOT FINDED
	else 
	{
		foundedInLastFrame = false;
		founded = false;
		if (foundedAtLeastOnce)
		{
			predicted = true;
			
			// set center and rect to predicted values
			center = predC;
			rect = predR;

			MA.insert(false);
		}
	}
}

void ObjectTracker::getInfo(bool &founded, bool &predicted, Point &c, Rect &r)
{
	founded = this->founded;
	predicted = this->predicted;
	c = center;
	r = rect;
}

void ObjectTracker::highlightObject(Mat &m, bool showCenter, bool showRect, bool showContour)
{
	if (!founded && !predicted)
	{
		return;
	}

	Scalar color;
	if (predicted)
	{
		color = Scalar(0, 0, 255); // red
	}
	if (founded)
	{
		color = Scalar(255, 0, 0); // blue
	}


	if (showCenter)
	{
		circle(m, center, 3, color, 2);
	}
	if (showRect)
	{
		rectangle(m, rect, color - Scalar(150,150,150), 4);
	}
	if (founded && showContour)
	{
		vector<vector<Point>> cnt;
		cnt.push_back(contour);
		drawContours(m, cnt, 0, color, 2);
	}
}

void ObjectTracker::showExtraInfo(bool accGraph, bool velGraph)
{
	// convert String to char*
	String strObjName = object.getName();
	const char *constObjName = strObjName.c_str();
	int len = strlen(constObjName) + 1;
	char * objName = new char[len];
	
	strcpy(objName,  constObjName);

	accGraph ? MA.printAccelerationGraph(objName) : 0;
	velGraph ? MA.printVelocityGraph(objName) : 0;

	delete[] objName;
}