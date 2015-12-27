#pragma once

#include "opencv2\highgui\highgui.hpp"

#include "MovementAnalysis.h"
#include "ObjectFinder.h"

using namespace cv;

class ObjectTracker
{
public:
	ObjectTracker();
	ObjectTracker(Object &obj);
	~ObjectTracker();

	// insert frame to tracker and automatically seacrch for object
	void insertFrame(Mat &frame);

	// get object state
	void getInfo(bool &founded, bool &predicted, Point &c, Rect &r);
	
	// draw object's center, rectangle and/or contour
	void highlightObject(Mat &m, bool showCenter, bool showRect, bool showContour);
	
	// print velocity and/or acceleration graph computed using MovementAnalysis class
	void showExtraInfo(bool accGraph = true, bool velGraph = true);
	
private:
	// object that tracker refers to
	// OBS: each tracker is in charge of a single object
	Object object;

	// used to search object in each frame
	ObjectFinder finder;

	// analyze object's trajectory
	// used to predict it's position
	MovementAnalysis MA;
	
	// frame in which the object is searched
	//  obs: only in worst cases object is finded in whole frame
	Mat frame;
	
	// variables requared in continous finding process:
	Point center;	//object's center
	Rect rect;	//object's rectangle
	vector<Point> contour;	//object's contour
	
	// object is finded in the current frame
	bool founded;

	// was finded in last frame
	bool foundedInLastFrame; 
	
	// object was finded at least once in previous frames
	bool foundedAtLeastOnce;

	// object isn't finded in the current frame
	// and it's position is predicted
	bool predicted; 

	// when object is founded it's contour coord are stored 
	//   so that in next frame, finder have to search just in 
	//	 in the place where object appeared last time
	//	 borded with a value
	Point p1; // represents minimum x and y coord of object's contour points
	Point p2; // represents maximum x and y coord of object's contour points

	//  number of pixels which extends last object position (p1  & p2)
	//    just if it was finded in last frame
	unsigned int border;
};

