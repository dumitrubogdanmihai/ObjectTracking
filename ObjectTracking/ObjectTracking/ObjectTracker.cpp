#include "Object.h"
#include "ObjectTracker.h"
#include "ObjectFinder.h"

#include "Helper.h"
#include "Finder.h"

/*
ObjectTracker::ObjectTracker(Mat &obj)
{
	this->obj = obj.clone();

	foundedInLastFrame = false;
	foundedAtLeastOnce = false;
}
*/
ObjectTracker::ObjectTracker(Object &obj)
{
	this->object = obj;

	foundedInLastFrame = false;
	foundedAtLeastOnce = false;
}

ObjectTracker::ObjectTracker()
{
	foundedInLastFrame = false;
	foundedAtLeastOnce = false;
}

void ObjectTracker::insertFrame(Mat &frame)
{
	Point pMin;
	Point pMax;

	Point predC;
	Rect predR;

	Mat searchedArea;

	foundedAtLeastOnce ? MA.getPred(predC, predR) : 0;

	// set searchedArea
	if (foundedInLastFrame)
	{
		Rect boundedRect; //small area from frame where the object should be
		Mat limiedFrame; //limit searching area to an small frame by bolding position of object in last frame

		boundedRect = boundPoints(p1, p2, 10, pMin, pMax, frame);
		limiedFrame = Mat(frame(boundedRect)).clone(); //selecting ROI

		searchedArea = limiedFrame;
	}
	else
	{
		searchedArea = frame;
	}
		
	// search object
	//contour = findObj(obj, searchedArea); // TRY to : find the object and extract his contour
	finder.find(object, frame, contour);
	
	// NOT FINDED
	if (objectFinded(contour, frame, pMin) == false || contour.size() < 4)
	{
		foundedInLastFrame = false;
		founded = false;
		if (foundedAtLeastOnce)
		{
			MA.insert(false);
			predicted = true;
			center = predC;
			rect = predR;
		}
	}
	else // FINDED
	{

		foundedInLastFrame = true;
		foundedAtLeastOnce == false ? foundedAtLeastOnce = true : 0;
		founded = true;
		predicted = false;

		//  decalate contour to point to the whole frame
		for (unsigned int j = 0; j < contour.size(); j++)
		{
			contour[j].x += pMin.x;
			contour[j].y += pMin.y;
		}

		// update last seen
		lastObjFoundedFrame = cropSelectedObject(frame, contour).clone();

		// update object's center and it's rectangle
		getRectAndCenter(contour, center, rect);
		
		 MA.insert(true, center, rect);
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
	//MA.printAccelerationGraph();
	//MA.printVelocityGraph();
}

ObjectTracker::~ObjectTracker()
{
}
