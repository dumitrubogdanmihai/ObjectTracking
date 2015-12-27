#include "Object.h"

Object::Object(Mat &objCap, const char *objectName)
{
	// clone capture
	cap = objCap.clone();

	captures.push_back(cap);

	// set name
	name = String(objectName);

	// computes keypoints and descriptors with ObjectFinder clas
	ObjectFinder finder;

	keypoints = finder.detectKeypoints(cap);
	descriptors = finder.calculateDescriptors(keypoints, cap);
	finder.extractor.compute(cap, keypoints, descriptors);
}

Object::~Object()
{
	cout << "Object class destructor was called" << endl;

	cap.release();
	captures.clear();
}

void Object::insertCapture(Mat &cap)
{
	captures.push_back(cap.clone());
}

void Object::updateCap()
{
	cap.release();
	cap = captures.back().clone();
}

void Object::print(char *windowName)
{
	imshow(windowName ? windowName : name, cap);
	waitKey(30);
}

String Object::getName()
{
	return name;
}