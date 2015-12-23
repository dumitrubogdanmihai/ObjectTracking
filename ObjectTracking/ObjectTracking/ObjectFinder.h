#pragma once

#include"stdafx.h"
#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/imgproc/imgproc.hpp" // convex 

#include "Object.h"

using namespace std;
using namespace cv;

class ObjectFinder
{
public:
	ObjectFinder();
	~ObjectFinder();

	vector<KeyPoint> detectKeypoints(Mat &m);
	Mat calculateDescriptors(vector<KeyPoint> keypoints, Mat &m);

	bool find(Object &obj, Mat &scene, vector<Point> &contourRez);

private:
	SurfFeatureDetector detector;
	SurfDescriptorExtractor extractor;
	FlannBasedMatcher matcher;

	friend class Object;
};

