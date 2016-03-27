// Object Tracker - OOP Project
// opencv 2.4.11

#include"stdafx.h"

#include<string.h>

#include "ObjectTracker.h"
#include "Helper.h" // crop object

using std::cout;

// left mouse button is pressed 
// selecting an object is in the process
bool clicked = false;

// main window where video captures are played 
//	to track an object from this window select his contour with the left mouse button pressed
char *mainWindow("Window");

// store each frame from video stream 
Mat frame;

// where all active trackers are stored
vector<ObjectTracker> trackers;

// to select the object using the mouse
void MouseCallBack(int event, int x, int y, int flags, void* userdata);

// interacts with user and sets a video capture
void setVideoCapture(VideoCapture &cap);

int main(int argc, char** argv)
{
	VideoCapture cap;
	setVideoCapture(cap);

	size_t frameCnt = 0;

	namedWindow(mainWindow);
	setMouseCallback(mainWindow, MouseCallBack, NULL);
	
	if (cap.isOpened() == false)
	{
		cout << "Capture missing!" << endl;
		return 1;
	}

	cap.read(frame);

	while (1)
	{

		imshow(mainWindow, frame);

		if (waitKey(30) == 27)
			break;

		// if the selection process is not in progress
		if (!clicked)
		{
			cout << endl  << "Frame " << ++frameCnt << endl;
			
			cap.read(frame);
			if (!frame.data)
				return 0;
			
			for (auto &t : trackers)
			{
				t.insertFrame(frame);
				t.highlightObject(frame, true, true, true);
				t.showExtraInfo();
			}
		}
	}

	return 0;
}

// 	used as a callback function to select object's contour from frame
void MouseCallBack(int event, int x, int y, int flags, void* userdata) 
{
	static vector<vector<Point>> contours(1);
	static Mat initialFrame;

	if (event == EVENT_LBUTTONDOWN)
	{
		cout << "Beginning process of selection for object number " << trackers.size() + 1 << endl;
		cout << "   Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
		
		clicked = true;
		
		initialFrame = frame.clone();
	}
	else if (event == EVENT_LBUTTONUP)
	{
		if (clicked == false)
			return;
		
		cout << "   Left button of the mouse is unclicked - position (" << x << ", " << y << ")" << endl << endl;
		
		Mat objectMat;
		char objectName[12] = "Object ";
		char cntStr[3];

		clicked = false;
		
		_itoa_s(trackers.size() + 1, cntStr, 3, 10);
		strcat_s(objectName, 12, cntStr);
				
		objectMat = cropSelectedObject(initialFrame, contours[0]);
				

		Object objectSelected(objectMat, objectName);
		objectSelected.print();

		// Instantiate a new tracker
		ObjectTracker newTracker(objectSelected);

		trackers.push_back(newTracker);
		
		contours[0].clear();
	}
	else if (clicked && event == EVENT_MOUSEMOVE)
	{
		if (contours[0].size() != 0)
		{
			line(frame, contours[0].back(), Point(x, y), Scalar(255, 0, 0), 3);
		}

		if (contours[0].size() == 0 || (contours[0][contours[0].size() - 1].x != x || contours[0][contours[0].size() - 1].y != y))
		{
			cout << "   Mouse move over the window - position (" << x << ", " << y << ")" << endl;
			contours[0].push_back(Point(x, y));
		}
	}
}

// computes an vector of pairs of files name and files path from within "d" directory 
void getFilesFromDir(const char* d, vector< pair< string, string > > & f)
{
	FILE* pipe = NULL;
	string pCmd = "dir /B /S " + string(d);
	char buf[256];

	if (NULL == (pipe = _popen(pCmd.c_str(), "rt")))
	{
		cout << "Shit" << endl;
		return;
	}

	while (!feof(pipe))
	{
		if (fgets(buf, 256, pipe) != NULL)
		{
			string fileName = string(buf).erase(0, strlen(d) + 1);
			fileName.pop_back();
			string filePath = string(buf);
			filePath.pop_back();
			f.push_back(make_pair(fileName, filePath));
		}

	}

	_pclose(pipe);
}

void setVideoCapture(VideoCapture &cap)
{
	vector< pair< string, string > > files;

	String capturesPath = __FILE__;
	capturesPath = capturesPath.substr(0, capturesPath.length() - 40) + "VideoCaptures";

	getFilesFromDir(capturesPath.c_str(), files);

	unsigned int option;
	unsigned int i;
	while (1)
	{

		cout << "Please select your video stream : " << endl << endl;
		cout << " 1. Webcam" << endl;
		cout << " 2. Enter file path" << endl;

		for (i = 0; i < files.size(); i++)
		{
			cout << " " << i + 3 << ". " << files[i].first << endl;
		}
		cout << endl << "Please insert your option : ";
		cin >> option;

		if (option > files.size() + 3)
		{
			cout << "Invalid input!" << endl;
			continue;
		}
		else
		{
			switch (option)
			{
			case 1:
			{
					  cap = VideoCapture(0);
					  break;
			}
			case 2:
			{
					  string filePath;
					  cout << " Please insert a path to a video file:" << endl;
					  cin >> filePath;
					  cap = VideoCapture(filePath);
					  break;
			}
			default:
			{
					   cout << " Open " << files[i - 3].second << endl;
					   cap = VideoCapture(files[i - 3].second);
			}
				break;
			}
		}

		if (!cap.isOpened())
		{
			cout << "Video capture couldn't be opened!" << endl;
			continue;
		}
		else
		{
			break;
		}
	}

	return;
}
