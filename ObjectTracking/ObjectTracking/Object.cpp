#include "Object.h"

Object::Object()
{
}

Object::Object(Mat &objCap)
{
	cap = objCap.clone();

	ObjectFinder finder;

	keypoints = finder.detectKeypoints(cap);
	
	descriptors = finder.calculateDescriptors(keypoints, cap);
	
	finder.extractor.compute(cap, keypoints, descriptors);
}

Object::~Object()
{
}
