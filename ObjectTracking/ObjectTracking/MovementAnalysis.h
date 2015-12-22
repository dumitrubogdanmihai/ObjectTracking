#pragma once

#include "opencv2/core/core.hpp"
#include <ctime>
#include "Kalman.h"

using namespace cv;

struct State
{
	vector<bool> valid;
	vector<Rect> r; // rectangle of object
	vector<Point> p; // point of center of object
	vector<Point> predictP; // predicted point
	vector<Rect> predictR; // predicted rectangle
	vector<double> a; // acceleration
	vector<double> v; // velocity
};

class MovementAnalysis
{
public:
	MovementAnalysis();
	void printAnalisys(Mat &m);
	void printVelocityGraph();
	void printAccelerationGraph();
	void insert(bool validPoint = false, Point &p = Point(0, 0), Rect &r = Rect(0, 0, 0, 0));
	void getPred(Point &c, Rect &r);
	
	~MovementAnalysis();

	Point lastP;
	double lastA;
	double lastV;
private:
	CKalmanFilter kf;

	double velocity(Point &p1, Point &p2, long long difT);
	double acceleration(double &v1, double &v2, long long difT);
	long long dT();

	State state;
};


void demoMA();