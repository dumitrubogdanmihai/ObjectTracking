
#include <string.h>
#include "MovementAnalysis.h"

#include "opencv2/highgui/highgui.hpp"
#include "opencv/cv.h"
#include "opencv/cxcore.h"

#include "GraphUtils.h"

#include "opencv2/highgui/highgui.hpp"// demo function

using namespace std;

MovementAnalysis::MovementAnalysis()
{
	state.a.clear();
	state.v.clear();
	state.p.clear();
	lastP = Point(0,0);
	lastA = 0;
	lastV = 0;
}

MovementAnalysis::~MovementAnalysis()
{
	cout << "MovementAnalysis class destructor was called!" << endl;
}

void MovementAnalysis::insert(bool validPoint, Point &p, Rect &r)
{
	Point predP;
	Rect predR;

	kf.getPredict(predP, predR);
	state.predictP.push_back(predP);
	state.predictR.push_back(predR);

	state.valid.push_back(validPoint);
	
	if (validPoint == false)
	{
		kf.update();

		// TO DO: using velocity and acceleration predict object's position, together with kalman filter 
		state.a.push_back(lastA);
		state.v.push_back(lastV);
		state.p.push_back(lastP);
	}
	else
	{
		kf.update(p, r);

		long long dt = dT();

		double v = velocity(lastP, p, dt);

		lastA = acceleration(lastV, v, dt);
		state.a.push_back(lastA);

		lastV = v;
		state.v.push_back(lastV);

		lastP = p;
		state.p.push_back(lastP);
	}
}

void MovementAnalysis::getPred(Point &c, Rect &r)
{
	c = state.predictP.back();
	r = state.predictR.back();
}

void MovementAnalysis::printAnalisys(Mat &m)
{
	Scalar color;
	Point p1;
	Point p2;

	printAccelerationGraph();
	printVelocityGraph();

	for (int i = 1; i < state.p.size(); i++)
	{

		if (state.valid[i])
		{
			color = Scalar(0, 255, 0);
		}
		else
		{
			color = Scalar(0, 0, 255);
		}

		p1 = (state.valid[i-1] ? state.p[i-1] : state.predictP[i-1]);
		p2 = (state.valid[i] ? state.p[i] : state.predictP[i]);

		circle(m, p2, 2, color, 1);
		line(m, p1, p2, color, 1);
		//rectangle(m, state.predictR[i], color, 2);
	}
	imshow("Analisys", m);
	waitKey(30);
}

void MovementAnalysis::printVelocityGraph(char *windowName)
{
	char wind[30] = "Velocity Graph";
	if(windowName != NULL)
	{
		strcat_s(wind, " - ");
		strcat_s(wind, windowName);
	}

	IplImage  * imgV = NULL;

	imgV = drawDoubleGraph(state.v, imgV);

	Mat velGraph = Mat(imgV);
	
	imshow(wind, velGraph);
	waitKey(1);

	cvReleaseImage(&imgV);
}

void MovementAnalysis::printAccelerationGraph(char *windowName)
{
	char wind[30] = "Acceleration Graph";
	if (windowName != NULL)
	{
		strcat_s(wind,  " - ");
		strcat_s(wind,  windowName);
	}

	IplImage  * imgA = NULL;

	imgA = drawDoubleGraph(state.a, imgA);

	Mat accGraph = Mat(imgA);

	imshow(wind, accGraph);
	waitKey(1);
	
	cvReleaseImage(&imgA);
}


long long MovementAnalysis::dT()
{
	static clock_t lastClock;
	clock_t dT = clock() - lastClock;
	lastClock = clock();

	return (long long)dT;
}

double MovementAnalysis::velocity(Point &p1, Point &p2, long long difT)
{
	double dst = cv::norm(p1 - p2);

	double speed = dst / difT * 1000;
	
	return speed;
}

double MovementAnalysis::acceleration(double &v1, double &v2, long long difT)
{
	double dst = v2 - v1;

	double accel = dst / difT * 1000;
	
	return accel;
}



// variables and functions used for demo 
int xM, yM;
bool clickedd;
void clickHandle(int event, int x, int y, int flags, void* userdata) // designed for manual selecting of object contour
{
	static vector<vector<Point>> contours(1);
	static Mat initialFrame;

	if (event == EVENT_LBUTTONDOWN)
	{
		clickedd = true;
	}
	else if (event == EVENT_LBUTTONUP)
	{
		clickedd = false;
	}
	if (clickedd)
	{
		xM = x;
		yM = y;
	}
}

void demoMA()
{
	cv::VideoCapture cap(0);
	Mat frame;
	cap.read(frame);
	frame = Mat(frame.size() * 2, frame.type());
	frame = Scalar(0, 0, 0);
	imshow("win", frame);
	waitKey(30);
	setMouseCallback("win", clickHandle, NULL);

	MovementAnalysis MA;

	while (1)
	{
		MA.insert(true, Point(xM, yM), Rect(0,0,0,0));
		MA.printAccelerationGraph();
		MA.printVelocityGraph();

		waitKey(30);
	}
}