#include "MovementAnalisys.h"

#include "opencv2/highgui/highgui.hpp"
#include "opencv/cv.h"
#include "opencv/cxcore.h"

#include "GraphUtils.h"

#include "opencv2/highgui/highgui.hpp"// demo function

using namespace std;

MovementAnalisys::MovementAnalisys()
{
	state.a.clear();
	state.v.clear();
	state.p.clear();
	lastP = Point(0,0);
	lastA = 0;
	lastV = 0;
}

void MovementAnalisys::printVelocityGraph()
{
	IplImage  * imgV = NULL;

	//cvSet(imgV, CV_RGB(255, 255, 255));

	imgV = drawDoubleGraph(state.v, imgV);

	Mat velGraph = Mat(imgV);
	
	imshow("Velocity Graph", velGraph);
	waitKey(1);

	cvReleaseImage(&imgV);
}

void MovementAnalisys::printAccelerationGraph()
{
	IplImage  * imgA = NULL;

	//cvSet(imgV, CV_RGB(255, 255, 255));

	imgA = drawDoubleGraph(state.a, imgA);

	Mat accGraph = Mat(imgA);

	imshow("Acceleration Graph", accGraph);
	waitKey(1);
	
	cvReleaseImage(&imgA);
}

long long MovementAnalisys::dT()
{
	static clock_t lastClock;
	clock_t dT = clock() - lastClock;
	lastClock = clock();

	return (long long)dT;
}
void MovementAnalisys::insert(Point &p)
{
	long long dt = dT();

	double v = velocity(lastP, p, dt);

	lastA = acceleration(lastV, v, dt);
	state.a.push_back(lastA);

	lastV = v;
	state.v.push_back(lastV);


	lastP = p;
	state.p.push_back(lastP);


	cout << "dt   " << dt << endl;
	cout << "lP   " << lastP << endl;
	cout << "lV   " << lastV << endl;
	cout << "lA   " << lastA << endl << endl;

}
MovementAnalisys::~MovementAnalisys()
{
}
double MovementAnalisys::velocity(Point &p1, Point &p2, long long difT)
{
	double dst = cv::norm(p1 - p2);
	//cout << "dist     = " << dst << "	pixels " << endl;
	//cout << "dif time = " << difT << "	  miliseconds " <<  endl;
	double speed = dst / difT * 1000;
	return speed;
}

double MovementAnalisys::acceleration(double &v1, double &v2, long long difT)
{
	double dst = v2 - v1;
	//cout << "dist     = " << dst << "	p/s " << endl;
	//cout << "dif time = " << difT << "	  miliseconds " << endl;
	double accel = dst / difT * 1000;
	return accel;
}





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

	MovementAnalisys MA;

	while (1)
	{
		MA.insert(Point(xM, yM));
		MA.printAccelerationGraph();
		MA.printVelocityGraph();

		waitKey(30);
	}
}