#include "ObjectFinder.h"
#include "Helper.h"

using std::cout;

ObjectFinder::ObjectFinder(int minHessian)
{
	detector = SurfFeatureDetector(minHessian);
}

ObjectFinder::~ObjectFinder()
{
	cout << "ObjectFinder class destructor was called!" << std::endl;
}

void ObjectFinder::find(Object const  &obj, Mat &scene, vector<Point> &contourRez)
{
	contourRez.clear();

	if (!obj.cap.data || !scene.data)
	{
		std::cout << " --(!) Error reading images " << std::endl;
	}

	//-- Step 1: Detect the keypoints using SURF Detector
	vector<KeyPoint> keypoints_scene = detectKeypoints(scene);

	//-- Step 2: Calculate descriptors (feature vectors)
	Mat descriptors_scene = calculateDescriptors(keypoints_scene, scene);

	extractor.compute(scene, keypoints_scene, descriptors_scene);
	
	//-- Step 3: Matching descriptor vectors using FLANN matcher
	std::vector< DMatch > matches;
	matcher.match(obj.descriptors, descriptors_scene, matches);

	double max_dist = 0; 
	double min_dist = 100;

	//-- Quick calculation of max and min distances between keypoints
	double dist;
	for (int i = 0; i < obj.descriptors.rows; i++)
	{
		dist = matches[i].distance;
		dist < min_dist ? min_dist = dist : 0;
		dist > max_dist ? max_dist = dist : 0;
	}

	//printf("-- Max dist : %f \n", max_dist);
	//printf("-- Min dist : %f \n\n", min_dist);

	//-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
	std::vector< DMatch > good_matches;

	//void filterMatches(vector< DMatch > &good_matches, vector< DMatch > matches);
	for (int i = 0; i < obj.descriptors.rows; i++)
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
	
	std::vector<Point2f> objV;
	std::vector<Point2f> sceneV;

	for (unsigned int i = 0; i < good_matches.size(); i++)
	{
		//-- Get the keypoints from the good matches
		objV.push_back(obj.keypoints[good_matches[i].queryIdx].pt);
		sceneV.push_back(keypoints_scene[good_matches[i].trainIdx].pt);
	}
	if (good_matches.size() < 5)
	{
		cout << "Good_matches vector size : " << good_matches.size() << endl << endl;
		return;
	}

	Mat H = findHomography(objV, sceneV, CV_RANSAC);

	//-- Get the corners from the image_1 ( the object to be "detected" )
	std::vector<Point2f> obj_corners(4);
	obj_corners[0] = cvPoint(0, 0); obj_corners[1] = cvPoint(obj.cap.cols, 0);
	obj_corners[2] = cvPoint(obj.cap.cols, obj.cap.rows); obj_corners[3] = cvPoint(0, obj.cap.rows);
	std::vector<Point2f> scene_corners(4);

	//for (int j = 0; j < scene_corners.size(); j++)
	//{
	//	scene_corners[j] = Point2f(0, 0);
	//}


	perspectiveTransform(obj_corners, scene_corners, H);


	//cout << "scene_corners " << endl << scene_corners << endl << " size:" << obj_corners.size() << endl << endl;

	contourRez.push_back(Point(scene_corners[0].x, scene_corners[0].y));
	contourRez.push_back(Point(scene_corners[1].x, scene_corners[1].y));
	contourRez.push_back(Point(scene_corners[2].x, scene_corners[2].y));
	contourRez.push_back(Point(scene_corners[3].x, scene_corners[3].y));

	return;
}

bool ObjectFinder::objectFinded(vector<Point> const &contour, Mat const  &frame, Point const &decal)
{
	if (contour.size() < 4)
	{
		return false;
	}
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

vector<KeyPoint> ObjectFinder::detectKeypoints(Mat &m)
{
	vector<KeyPoint> keyp;

	detector.detect(m, keyp);

	return keyp;
}

Mat ObjectFinder::calculateDescriptors(vector<KeyPoint> &keypoints, Mat &m)
{
	Mat descriptors;

	extractor.compute(m, keypoints, descriptors);

	return descriptors;
}