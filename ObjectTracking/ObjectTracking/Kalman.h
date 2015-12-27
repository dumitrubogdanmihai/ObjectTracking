#pragma once

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp" // KalmanFilter class

using namespace cv;

// demo using mouse movement
//	if space is pressed consider mouse position as valid 
//	otherwise predict it's position
void demoKF();

// kalman filter class designed for object position predictions 
class CKalmanFilter
{
public:
	CKalmanFilter(int stateSize = 6, int measSize = 4, int contrSize = 0, unsigned int type = CV_32F);
	~CKalmanFilter();

	// get predicted values
	void getPredict(Point &objectPred, Rect &objectPredBox);
	
	// update with new values 
	void update(Point object = Point(0, 0), Rect objectBox = Rect(0, 0, 0, 0));
	
	// print object's trajectory and it's prediction
	void printTrajectories(Size &sz);

private:
	// varoables used for kalman filter
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
	
	// where object's coordinates are stored
	Vector<Point> objV;
	Vector<Point> aproxObjV;
};

