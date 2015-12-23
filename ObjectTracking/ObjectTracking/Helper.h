#include "opencv2/highgui/highgui.hpp"

using namespace std;
using namespace cv;

long long difTime();
long long difTime2();
void getRectAndCenter(const vector<Point> &contour, Point &centerC, Rect &rectC);
double areaOf(Point &p1, Point &p2);
Rect boundPoints(Point p1, Point p2, int exp, Point &pMinRez, Point &pMaxRez, Mat &frame);
void minNormalRect(vector<Point> &contour, Point& min, Point &max);
Mat cropSelectedObject(Mat & source, vector<Point> contour, bool printObjRect = false);
bool intersection(Point2f o1, Point2f p1, Point2f o2, Point2f p2);
void drawDetails(Mat &frame, vector<vector<Point>> &cntr, Scalar &cntrClr, Rect &rect, Point &p1, Point &p2, Point &pmin, Point &pmax);