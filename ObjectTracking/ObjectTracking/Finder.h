#include"stdafx.h"
#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/imgproc/imgproc.hpp" // convex Hull


using namespace std;
using namespace cv;

bool objectFinded(vector<Point>contour, Mat &frame, Point &decal);
vector<Point> findObj(Mat &img_object, Mat &img_scene);