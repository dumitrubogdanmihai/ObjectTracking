#pragma once

#include "opencv2\highgui\highgui.hpp"
#include "MovementAnalysis.h"

using namespace std;
using namespace cv;

class ObjectTracker
{
public:
	ObjectTracker();
	//ObjectTracker(Mat &obj);
	ObjectTracker(Object &obj);
	void insertFrame(Mat &frame);
	void getInfo(bool &founded, bool &predicted, Point &c, Rect &r);
	void highlightObject(Mat &m, bool showCenter, bool showRect, bool showContour);
	void showExtraInfo();
	~ObjectTracker();

	bool foundedAtLeastOnce; //if was finded at least once in previous frames

private:
	MovementAnalysis MA;
	ObjectFinder finder;
	Object object;

	Point center;	//object's center
	Rect rect;	//object's rectangle
	vector<Point> contour;	//object's contour

	Mat obj;	//object to be founded
	

	Mat lastObjFoundedFrame;	//object to be founded
	Mat frame;	//whole frame

	bool foundedInLastFrame; //if was finded in last frame
	
	bool founded;
	bool predicted;

	//to repalce p1 & p2
	//Rect lastFrame;

	Point p1;
	Point p2;

};

