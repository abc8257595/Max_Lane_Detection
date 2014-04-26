/*------------------------------------------------------------------------------------------*\
	Add up number on lines that are found within a threshold of a given rho,theta and 
	use that to determine a score.  Only lines with a good enough score are kept. 

	Calculation for the distance of the car from the center.  This should also determine
	if the road in turning.  We might not want to be in the center of the road for a turn. 
	
	Several other parameters can be played with: min vote on houghp, line distance and gap.  Some
	type of feed back loop might be good to self tune these parameters. 

	We are still finding the Road, i.e. both left and right lanes.  we Need to set it up to find the
	yellow divider line in the middle. 

	Added filter on theta angle to reduce horizontal and vertical lines. 

	Added image ROI to reduce false lines from things like trees/powerlines
\*------------------------------------------------------------------------------------------*/

#include <highgui/highgui.hpp>
#include <objdetect/objdetect.hpp>
#include <imgproc/imgproc.hpp>
#include <iostream>
#include <stdio.h>
#include <vector>
#include "linefinder.h"

#define PI 3.1415926

using namespace std;
using namespace cv;

    void help(char** argv) {
        cout << "This program aim to detect the Road Lane\n"
            "Usage:\n./" << argv[0] << " <video device number>\n"
            << "q,Q,esc -- quit\n"
            << "space   -- save frame\n\n"
            << "To find the video device number, try ls /dev/video* \n"
            << "You may also pass a video file, like my_vide.avi instead of a device number"
            << endl;
    }

int main(int argc, char** argv) {

    if (argc != 2) {
        help(argv);
        return -1;
    }
    std::string arg = argv[1];
	
    VideoCapture capture(arg); //try to open string, this will attempt to open it as a video file
    if (!capture.isOpened()) //if this fails, try to open as a video camera, through the use of an integer param
        capture.open(atoi(arg.c_str()));
    if (!capture.isOpened()) {
        cerr << "Failed to open a video device or video file!\n" << endl;
        help(argv);
        return -1;
    }

    int n = 0 ; //image NO
    char filename[200];
    string window_name = "Lane Detection";
    cout << "press space to save a picture. q or esc to quit" << endl;
    namedWindow(window_name, CV_WINDOW_AUTOSIZE); //resizable window;
    Mat frame;
    while(1){
        capture >> frame;
        if (frame.empty())
            break;
        imshow(window_name, frame);
        char key = (char)waitKey(10); //delay N millis, usually long enough to display and capture input
        switch (key) {
		    case 'q':
		    case 'Q':
		    case 27: //escape key
		        return 0;
		    case ' ': //Save an image
		        sprintf(filename,"filename%.3d.jpg",n++);
		        imwrite(filename,frame);
		        cout << "Saved " << filename << endl;
		        break;
		    default:
		        break;
        }
        // main function
    }
    return 0;
}
