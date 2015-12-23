#pragma once


#include "opencv2\highgui\highgui.hpp"
#include "opencv2\features2d\features2d.hpp"

#include "ObjectFinder.h"

using namespace std;
using namespace cv;

class Object
{
public:
	Object();
	Object(Mat &objCap);
	~Object();
private:
	Mat cap;
	vector <Mat> captures;
	vector<KeyPoint> keypoints;
	Mat descriptors;

	friend class ObjectFinder;
};

