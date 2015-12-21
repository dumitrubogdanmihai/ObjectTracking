#pragma once

#include "opencv2/core/core.hpp"
#include <ctime>

using namespace cv;

struct State
{
	vector<Point> p;
	vector<double> a;
	vector<double> v;
};

class MovementAnalisys
{
public:
	MovementAnalisys();
	void printVelocityGraph();
	void printAccelerationGraph();
	void insert(Point &p);
	~MovementAnalisys();
private:
	double velocity(Point &p1, Point &p2, long long difT);
	double acceleration(double &v1, double &v2, long long difT);
	long long dT();

	State state;
	Point lastP;
	double lastA;
	double lastV;
};


void demoMA();