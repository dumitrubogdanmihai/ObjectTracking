#include "opencv2/highgui/highgui.hpp"

using namespace cv;

// computes from an contour it's center and minimum rectangle that can cover all of it's points
void getRectAndCenter(const vector<Point> &contour, Point &centerC, Rect &rectC);

// returns area of an rectangle formed by 2 points
double areaOf(Point &p1, Point &p2);

// expand p1 and p2 with exp pixels limited to frame's size, 
//	resulted points are returned by pMinRez and pMaxRez
Rect boundPoints(Point p1, Point p2, int exp, Point &pMinRez, Point &pMaxRez, Mat &frame);

// crop object from source based on a vector of points
Mat cropSelectedObject(Mat & source, vector<Point> &contour, bool applyMask = false);

// check if lines formed by o1, p1 and o2, p2 are intersecting
bool intersection(Point2f o1, Point2f p1, Point2f o2, Point2f p2);
