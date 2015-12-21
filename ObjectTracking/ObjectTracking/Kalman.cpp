#include "stdafx.h"
#include "Kalman.h"

void CKalmanFilter::update(Point object, Rect objectBox)
{
	if (objectBox == Rect(0, 0, 0, 0))
	{
		notFoundCount++;
		//std::cout << "notFoundCount:" << notFoundCount << endl;
		if (notFoundCount >= 100)
		{
			found = false; // useless
		}
		/*else
		kf.statePost = state;*/
	}
	else
	{

		static double precTick = ticks;
		double ticks = (double)cv::getTickCount();
		dT = (ticks - precTick) / cv::getTickFrequency(); //seconds
		cout << dT << endl;
		cout << ticks << endl;
		cout << precTick << endl;

		objV.push_back(object);

		notFoundCount = 0;
		meas.at<float>(0) = objectBox.x + objectBox.width / 2;
		meas.at<float>(1) = objectBox.y + objectBox.height / 2;
		meas.at<float>(2) = (float)objectBox.width;
		meas.at<float>(3) = (float)objectBox.height;

		if (!found) // First detection!
		{
			// >>>> Initialization
			kf.errorCovPre.at<float>(0) = 1; // px
			kf.errorCovPre.at<float>(7) = 1; // px
			kf.errorCovPre.at<float>(14) = 1;
			kf.errorCovPre.at<float>(21) = 1;
			kf.errorCovPre.at<float>(28) = 1; // px
			kf.errorCovPre.at<float>(35) = 1; // px

			state.at<float>(0) = meas.at<float>(0);
			state.at<float>(1) = meas.at<float>(1);
			state.at<float>(2) = 0;
			state.at<float>(3) = 0;
			state.at<float>(4) = meas.at<float>(2);
			state.at<float>(5) = meas.at<float>(3);
			// <<<< Initialization

			found = true;
		}
		else
			kf.correct(this->meas); // Kalman Correction
	}
}
void CKalmanFilter::predict(Point &objectPred, Rect &objectPredBox)
{
	static double precTick = ticks;
	double ticks = (double)cv::getTickCount();
	dT = (ticks - precTick) / cv::getTickFrequency(); //seconds

	// >>>> Matrix A
	kf.transitionMatrix.at<float>(2) = dT;
	kf.transitionMatrix.at<float>(9) = dT;
	// <<<< Matrix A

	//std::cout << "dT: " << dT << endl;

	state = kf.predict();
	//std::cout << "State post:" << endl << state << endl;

	objectPredBox.width = state.at<float>(4);
	objectPredBox.height = state.at<float>(5);
	objectPredBox.x = state.at<float>(0) - objectPredBox.width / 2;
	objectPredBox.y = state.at<float>(1) - objectPredBox.height / 2;

	objectPred.x = state.at<float>(0);
	objectPred.y = state.at<float>(1);

	aproxObjV.push_back(objectPred);
}

void CKalmanFilter::printTrajectories(Size &sz)
{
	Scalar(1, 1, 1);
	Mat traj(sz, CV_8UC3);
	traj = Scalar(0, 0, 0);
	if (aproxObjV.size() > 1)
	{
		for (int i = 0; i < aproxObjV.size() - 1; i++)
		{
			line(traj, aproxObjV[i], aproxObjV[i + 1], Scalar(0, 255 * i / aproxObjV.size(), 0), 2);
		}
	}
	if (objV.size() > 1)
	{
		for (int i = 0; i < objV.size() - 1 && objV.size(); i++)
		{
			line(traj, objV[i], objV[i + 1], Scalar(0, 0, 255 * i / objV.size()), 2);
		}
	}
	

	if (objV.size() > 1)
	{
		circle(traj, objV.back(), 3, Scalar(0, 0, 255), 2);
	}
	imshow("Trajectories", traj);
	
	waitKey(30);
}

CKalmanFilter::CKalmanFilter(int stateSize, int measSize, int contrSize, unsigned int type)
{
	ticks = 0;
	found = false;
	objFinded = false;
	notFoundCount = 0;
	this->stateSize = stateSize;
	this->measSize = measSize;
	this->contrSize = contrSize;
	this->type = type;

	state = Mat(stateSize, 1, type);  // [x,y,v_x,v_y,w,h]
	meas = Mat(measSize, 1, type);  // [z_x,z_y,z_w,z_h]

	kf = KalmanFilter(stateSize, measSize, contrSize, type);

	cv::setIdentity(kf.transitionMatrix);

	kf.measurementMatrix = cv::Mat::zeros(measSize, stateSize, type);
	kf.measurementMatrix.at<float>(0) = 1.0f;
	kf.measurementMatrix.at<float>(7) = 1.0f;
	kf.measurementMatrix.at<float>(16) = 1.0f;
	kf.measurementMatrix.at<float>(23) = 1.0f;

	kf.processNoiseCov.at<float>(0) = 1e-2;
	kf.processNoiseCov.at<float>(7) = 1e-2;
	kf.processNoiseCov.at<float>(14) = 5.0f;
	kf.processNoiseCov.at<float>(21) = 5.0f;
	kf.processNoiseCov.at<float>(28) = 1e-2;
	kf.processNoiseCov.at<float>(35) = 1e-2;

	cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(1e-1));
}



// kalman test

bool detect(Point &object, Rect &objectBox)
{
	POINT mousePos;
	GetCursorPos(&mousePos);
	Point p(mousePos.x, mousePos.y);

	if (GetAsyncKeyState(VK_SPACE))
	{
		object = p;
		objectBox = Rect(p.x, p.y, 10, 10);
		return true;
	}
	return false;
}

void demo()
{
	CKalmanFilter kfilter = CKalmanFilter();

	bool found = false;
	bool objFinded = false;
	Point object;
	Rect objectBox;
	Point predP;
	Rect predR;

	Mat img = Mat(1024, 720, CV_8UC3);

	while (1)
	{
		while (GetAsyncKeyState(VK_UP));
		found ? kfilter.predict(predP, predR) : 0;

		objFinded = detect(object, objectBox);

		objFinded ? found = true : 0;

		objFinded ? kfilter.update(object, objectBox) : kfilter.update();

		img = Scalar(0, 0, 0);

		objFinded ? rectangle(img, Rect(object.x, object.y, 10, 10), Scalar(215, 215, 215), 4) : 0;
		found ? rectangle(img, Rect(predP.x, predP.y, 10, 10), Scalar(0, 0, 255), 4) : 0;

		imshow("IMG", img);
		waitKey(30);
		GetAsyncKeyState(VK_DOWN) ? kfilter.printTrajectories(img.size()) : 0;

	}
}