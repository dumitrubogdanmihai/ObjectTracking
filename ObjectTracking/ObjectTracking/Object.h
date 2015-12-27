#pragma once

#include "opencv2\features2d\features2d.hpp"
#include "ObjectFinder.h"

using namespace cv;

// this class represents objects that must be tracked
class Object
{
public:
	Object(){};
	Object(Mat &objCap, const char *objectName);
	~Object();

	// insert a new captures to captures vector
	void insertCapture(Mat &cap);

	// update object's capture on which the search is applied 
	//	 with last finded capture of it
	void updateCap();

	// print object capture on a window with it's name given by the parameter
	//   if it will be called without parameter, windowName will be object's name
	void print(char *windowName = NULL);

	String getName();
private:
	// object's main capture 
	//   on which the search is applied
	Mat cap; 

	// stores all captures finded 
	vector <Mat> captures;
	
	// object's name
	String name;

	// those variable are computed just once in constructor of this class
	// and are eventually used afterwards in ObjectFinder class
	vector<KeyPoint> keypoints;
	Mat descriptors;

	// to acces keypoints and descriptors variables
	friend class ObjectFinder;
};

