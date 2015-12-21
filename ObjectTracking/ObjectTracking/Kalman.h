#pragma once
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"
#include <iostream>
#include <Windows.h>

#define drawCross( center, color, d, img )                                 \
	

using namespace cv;
using namespace std;

void demo();

class CKalmanFilter
{
public:
	void predict(Point &objectPred, Rect &objectPredBox);
	void update(Point object = Point(0, 0), Rect objectBox = Rect(0, 0, 0, 0));
	void printTrajectories(Size &sz);
	CKalmanFilter(int stateSize = 6, int measSize = 4, int contrSize = 0, unsigned int type = CV_32F);
private:
	KalmanFilter kf;

	int stateSize;
	int measSize;
	int contrSize;
	unsigned int type;

	double ticks;
	bool found;
	bool objFinded;

	int notFoundCount;

	double dT;
	Mat state;  // [x,y,v_x,v_y,w,h]
	Mat meas;    // [z_x,z_y,z_w,z_h]
	Point object;
	Rect objectBox;

	Vector<Point> objV;
	Vector<Point> aproxObjV;
};


class CKalmanFilterSimple
{
public:
	void predict(Point &objectPred);
	void update(bool valid = false, Point pt = Point(0, 0));
	void printTrajectories(Size &sz);
	CKalmanFilterSimple(int stateSize = 6, int measSize = 4, int contrSize = 0, unsigned int type = CV_32F);
private:
	KalmanFilter kf;

	int stateSize;
	int measSize;
	int contrSize;
	unsigned int type;

	double ticks;
	bool found;
	bool objFinded;

	int notFoundCount;

	double dT;
	Mat state;  // [x,y,v_x,v_y,w,h]
	Mat meas;    // [z_x,z_y,z_w,z_h]
	Point object;

	Vector<Point> objV;
	Vector<Point> aproxObjV;
};