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

// forward declaration 
//  because Object and ObjectFinder classes includes each other
class Object;

class ObjectFinder
{
public:
	ObjectFinder(int minHessian = 400);
	~ObjectFinder();

	// find obj in scene and computes it's resulted contour from scene
	void find(Object const &obj, Mat &scene, vector<Point> &contourRez);

	// checks if an contour resulted from find() funtion is valid
	//   with other words if the ofject was finded
	bool objectFinded(vector<Point>  const &contour, Mat  const &frame, Point  const &decal);

	// return keypoints from an image
	vector<KeyPoint> detectKeypoints(Mat &m);

	// return descriptors of keypoints from an image
	Mat calculateDescriptors(vector<KeyPoint>  &keypoints, Mat &m);
	
private:
	//Class for extracting Speeded Up Robust Features
	SurfFeatureDetector detector;

	//Class for extracting descriptors
	SurfDescriptorExtractor extractor;

	//Fast Approximate Nearest Neighbor Search Library 
	//  for quick features matching
	FlannBasedMatcher matcher;

	friend class Object;
};

