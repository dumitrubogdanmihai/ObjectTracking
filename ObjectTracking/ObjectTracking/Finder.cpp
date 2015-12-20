#include "stdafx.h"
#include"Finder.h"
#include "Helper.h"

void detectKeypoints(Mat &obj, Mat &sce, vector<KeyPoint> & keyp_obj, vector<KeyPoint> & keyp_sc)
{
	int minHessian = 400;

	SurfFeatureDetector detector(minHessian);

	detector.detect(obj, keyp_obj);
	detector.detect(sce, keyp_sc);
};


bool objectFinded(vector<Point>contour, Mat &frame, Point &decal)
{
	if (contourArea(contour) < 15)
	{
		cout << "Contour area < 15" << endl << endl;
		return false;
	}

	if (intersection(contour[0], contour[1], contour[2], contour[3]))
	{
		cout << "Diagonals are intersecting" << endl << endl;
		//cout << contour << endl;
		return false;
	}

	if(contour.size() < 4 )
	{
		return false;
	}

	for (int j = 0; j < 4; j++)
	{
		if (contour[j].x + decal.x < 0)
		{
			cout << "Object unfinded! " << endl;
			cout << " Contour out of limits : ";
		//	cout << " x= " << contour[j].x + decal.x << endl << endl;
			return false;
		}
		else if (contour[j].x + decal.x > frame.cols)
		{
			cout << "Object unfinded! " << endl;
			cout << " Contour out of limits : ";
		//	cout << " x= " << contour[j].x + decal.x << endl << endl;
			return false;
		}
		else if (contour[j].y + decal.y < 0)
		{
			cout << "Object unfinded! " << endl;
			cout << " Contour out of limits : ";
		//	cout << " y= " << contour[j].y + decal.y << endl << endl;
			return false;
		}
		else if (contour[j].y + decal.y > frame.rows)
		{
			cout << "Object unfinded! " << endl;
			cout << " Contour out of limits : ";
		//	cout << " y= " << contour[j].y + decal.y << endl << endl;
			return false;
		}
	}
	
	cout << "Object finded! " << endl;
	return true;
}


vector<Point> findObj(Mat &img_object, Mat &img_scene)
{
	vector<Point> objectFrame; // returned val;


	if (!img_object.data || !img_scene.data)
		std::cout << " --(!) Error reading images " << std::endl;

	//-- Step 1: Detect the keypoints using SURF Detector
	vector<KeyPoint> keypoints_object, keypoints_scene;
	detectKeypoints(img_object, img_scene, keypoints_object, keypoints_scene);


	//-- Step 2: Calculate descriptors (feature vectors)
	SurfDescriptorExtractor extractor;

	Mat descriptors_object, descriptors_scene;

	extractor.compute(img_object, keypoints_object, descriptors_object);
	extractor.compute(img_scene, keypoints_scene, descriptors_scene);


	//-- Step 3: Matching descriptor vectors using FLANN matcher
	FlannBasedMatcher matcher;
	std::vector< DMatch > matches;
	matcher.match(descriptors_object, descriptors_scene, matches);

	double max_dist = 0; double min_dist = 100;

	//-- Quick calculation of max and min distances between keypoints
	for (int i = 0; i < descriptors_object.rows; i++)
	{
		double dist = matches[i].distance;
		if (dist < min_dist) min_dist = dist;
		if (dist > max_dist) max_dist = dist;
	}

	//printf("-- Max dist : %f \n", max_dist);
	//printf("-- Min dist : %f \n\n", min_dist);

	//-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
	std::vector< DMatch > good_matches;

	//void filterMatches(vector< DMatch > &good_matches, vector< DMatch > matches);
	for (int i = 0; i < descriptors_object.rows; i++)
	{
		if (matches[i].distance < 3 * min_dist)
		{
			good_matches.push_back(matches[i]);
		}
	}

	/*drawMatches(img_object, keypoints_object, img_scene, keypoints_scene,
	good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
	vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
	*/

	//-- Localize the object
	std::vector<Point2f> obj;
	std::vector<Point2f> scene;

	for (int i = 0; i < good_matches.size(); i++)
	{
		//-- Get the keypoints from the good matches
		obj.push_back(keypoints_object[good_matches[i].queryIdx].pt);
		scene.push_back(keypoints_scene[good_matches[i].trainIdx].pt);
	}
	if (good_matches.size() < 5)
	{
		cout << "good_matches size : " << good_matches.size() << endl << endl;
		return objectFrame;
	}
	
	Mat H = findHomography(obj, scene, CV_RANSAC);

	//-- Get the corners from the image_1 ( the object to be "detected" )
	std::vector<Point2f> obj_corners(4);
	obj_corners[0] = cvPoint(0, 0); obj_corners[1] = cvPoint(img_object.cols, 0);
	obj_corners[2] = cvPoint(img_object.cols, img_object.rows); obj_corners[3] = cvPoint(0, img_object.rows);
	std::vector<Point2f> scene_corners(4);
	
	//for (int j = 0; j < scene_corners.size(); j++)
	//{
	//	scene_corners[j] = Point2f(0, 0);
	//}


	perspectiveTransform(obj_corners, scene_corners, H);
	

	//cout << "scene_corners " << endl << scene_corners << endl << " size:" << obj_corners.size() << endl << endl;

	objectFrame.push_back(Point(scene_corners[0].x, scene_corners[0].y));
	objectFrame.push_back(Point(scene_corners[1].x, scene_corners[1].y));
	objectFrame.push_back(Point(scene_corners[2].x, scene_corners[2].y));
	objectFrame.push_back(Point(scene_corners[3].x, scene_corners[3].y));
	
	return objectFrame;
}