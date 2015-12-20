#include "stdafx.h"
#include "Helper.h"
#include <iostream>
#include "opencv2/imgproc/imgproc.hpp" // contourArea


using namespace cv;
using namespace std;


int areaOf(Point &p1, Point &p2)
{

	vector<Point> contour;
	contour.push_back(p1);
	contour.push_back(Point(p2.x, p1.y));
	contour.push_back(p2);
	contour.push_back(Point(p1.x, p2.y));
	return contourArea(contour);

}

Rect boundPoints(int exp, Point& p1, Point& p2, Point &pMinRez, Point &pMaxRez, Mat &frame)
{
	if (p1 == p2)
	{
		pMinRez = Point(0, 0);
		pMaxRez = Point(frame.cols, frame.rows);
	}
	//  if p2 == (0,0)
	else if (p2 == Point(0, 0))
	{
		pMinRez = Point(0, 0);
		pMaxRez = Point(frame.cols, frame.rows);
	}
	else
	{
		pMinRez.x = (p1.x - exp < 0 ? 0 : p1.x - exp);
		pMinRez.y = (p1.y - exp < 0 ? 0 : p1.y - exp);

		pMaxRez.x = (p2.x + exp > frame.cols ? frame.cols : p2.x + exp);
		pMaxRez.y = (p2.y + exp > frame.rows ? frame.rows : p2.y + exp);
	}

	if (areaOf(p1, p2) < 15)
	{
		pMinRez = Point(0, 0);
		pMaxRez = Point(frame.cols, frame.rows);
	}
	return Rect(pMinRez, pMaxRez);
}

void minNormalRect(vector<Point> &contour, Point& min, Point &max)
{
	int xMin = contour[0].x;
	int xMax = 0;
	int yMin = contour[0].y;
	int yMax = 0;
	for (int j = 0; j < contour.size(); j++)
	{
		contour[j].x < xMin ? xMin = contour[j].x : 0;
		contour[j].x > xMax ? xMax = contour[j].x : 0;
		contour[j].y < yMin ? yMin = contour[j].y : 0;
		contour[j].y > yMax ? yMax = contour[j].y : 0;
	}
	min = Point(xMin, yMin);
	max = Point(xMax, yMax);
}

Mat cropSelectedObject(Mat & source, vector<Point> contour)
{
	// computing convexHull of contour points
	vector<vector<Point>> hull(1);
	convexHull(Mat(contour), hull[0], false);

	// apply mask
	Mat src = source.clone();
	Mat mask = Mat::zeros(src.size(), src.type());
	Mat maskedObj = Mat::zeros(src.size(), src.type());
	drawContours(mask, hull, 0, Scalar(255, 255, 255), CV_FILLED, 8, vector<Vec4i>(), 0, Point());

	//src.copyTo(maskedObj, mask);
	maskedObj = src.clone();

	// Find min square
	Point p1, p2;
	minNormalRect(hull[0], p1, p2);

	Mat croppedObj = maskedObj(Rect(p1, p2));

	return croppedObj;
}

bool in(int n, int a, int b)
{
	if (n >= a && n <= b)
		return true;
	if (n >= b && n <= a)
		return true;
	return false;
}

bool intersection(Point2f o1, Point2f p1, Point2f o2, Point2f p2)
{
	Point2f x = o2 - o1;
	Point2f d1 = p1 - o1;
	Point2f d2 = p2 - o2;

	float cross = d1.x*d2.y - d1.y*d2.x;
	if (abs(cross) < /*EPS*/1e-8)
		return false;

	double t1 = (x.x * d2.y - x.y * d2.x) / cross;
	Point2f r = o1 + d1 * t1;

	if (in(r.x, o1.x, p1.x) && in(r.y, o1.y, p1.y))
		return true;
	if (in(r.x, o2.x, p2.x) && in(r.y, o2.y, p2.y))
		return true;
	return false;
}


void drawDetails(Mat &frame, vector<vector<Point>> &cntr, Scalar &cntrClr, Rect &rect, Point &p1, Point &p2, Point &pmin, Point &pmax)
{
	rectangle(frame, rect, Scalar(255, 0, 0), 2);

	drawContours(frame, cntr, 0, cntrClr, 2);

	putText(frame, "p1", p1, FONT_HERSHEY_COMPLEX_SMALL, 0.5, cvScalar(200, 200, 250), 1, CV_AA);
	putText(frame, "p2", p2, FONT_HERSHEY_COMPLEX_SMALL, 0.5, cvScalar(200, 200, 250), 1, CV_AA);
	putText(frame, "pMin", pmin, FONT_HERSHEY_COMPLEX_SMALL, 0.5, cvScalar(200, 200, 250), 1, CV_AA);
	putText(frame, "pMax", pmax, FONT_HERSHEY_COMPLEX_SMALL, 0.5, cvScalar(200, 200, 250), 1, CV_AA);
}