#pragma once

#include <ctime>
#include "opencv2/core/core.hpp"

#include "Kalman.h"

using namespace cv;

struct State
{
	
	// rectangle of object
	vector<Rect> r; 

	// point of center of object
	vector<Point> p; 
	
	// point and rect are valid
	vector<bool> valid;

	// predicted point
	vector<Point> predictP;

	// predicted rectangle
	vector<Rect> predictR;
	
	// acceleration
	vector<double> a; 

	// velocity
	vector<double> v; 
};

class MovementAnalysis
{
public:
	MovementAnalysis();
	~MovementAnalysis();

	// insert new data to analyzer
	void insert(bool validPoint = false, Point &p = Point(0, 0), Rect &r = Rect(0, 0, 0, 0));
	
	// get predicted center and rectangle
	void getPred(Point &c, Rect &r);
	
	// print velocity and acceleration graph and trajectoy of object
	void printAnalisys(Mat &m);

	void printVelocityGraph(char *windowName = NULL);
	void printAccelerationGraph(char *windowName = NULL);
	
private:
	State state;
	
	CKalmanFilter kf;

	Point lastP;	// last point
	double lastV;	// last velocity
	double lastA;	// last acceleration

	// computes velocity using 2 points and time elapsed between them
	// p1 = origin
	// p2 = destination
	// unit of measurement = pixel / second
	double velocity(Point &p1, Point &p2, long long difT);

	// computes acceleration using velocities from 2 point and time elapsed between them
	// v1  velocity from origin point
	// v2  velocity from destination point
	// unit of measurement = pixel / second ^ 2
	double acceleration(double &v1, double &v2, long long difT);

	// returns the time elapsed from last call of this function
	// unit of measurement =  milisecond
	long long dT();
};

// demo using mouse movement
void demoMA();