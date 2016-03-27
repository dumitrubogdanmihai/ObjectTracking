# ObjectTracker

## Description
 This is a faculty project for the OOP course.
 The project represents  an object tracker for non-static cameras using C++14 and OpenCV 2.4.11

## How to use it
 All you have to do is to select with the mouse object's contour that you want to be tracked
 You can track multiple object simultaneously if your cpu can handle that

## Under the hood
 After selecting object's contour with the mouse and by releasing the mouse’s button an ObjectTracker will be instantiated based on an instance of the Object class Object instance is formed by object's frame and object's name.

 ObjectTracker uses ObjectFinder to search for that object in a  Region Of Interest as much as possible, but if is not found it will consider whole frame.

 ObjectTracker interacts with MovementAnalisys to analize object's movement ( velocity and acceleration ) and predict object's position whenever it couldn't be found.
 An acceleration graph and velocity graph of object positions will also be printed.
 
## UML Diagram 
[Diagram]( http://i.imgur.com/k0L8Cpy.jpg)

## Requirements 
 1. Windows OS 
 2. Visual Studio '13
 3. OpenCV 2.4.11
 
## Installation 
 1. Download OpenCV 2.4.11 for Windows directly from [here](https://sourceforge.net/projects/opencvlibrary/files/opencv-win/2.4.11/opencv-2.4.11.exe/download) or [here](http://opencv.org/downloads.html)
 2. Install opencv
 3. Set a path variable to opencv with 
  -Variable name: OPENCV_DIR
  -Variable value: C:\opencv\opcv2.4.11\opencv\build\x86\vc12
 4. Import OpenCV_Debug.props to visual studio to configure all links with library
 5. Hit build and run to have fun 
 
 If you have any trouble to configure opencv you can consult a full guide from [here](https://marcomuraresearch.wordpress.com/2015/04/16/install-opencv-visual-studio/).



