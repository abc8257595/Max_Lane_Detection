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
            << "space   -- pause\n"
            << "enter   -- save frame\n\n"
            << "To find the video device number, try ls /dev/video* \n"
            << "You may also pass a video file, like my_vide.avi instead of a device number"
            << endl;
}

int main(int argc, char** argv) {

//**************************************** 标准化输入输出(前奏) *************************************************//
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
    cout << "press enter to save a picture. q or esc to quit" << endl;
    namedWindow(window_name, CV_WINDOW_AUTOSIZE); //resizable window;
    
    Mat frame;
    double dWidth = capture.get(CV_CAP_PROP_FRAME_WIDTH);
    double dHeight = capture.get(CV_CAP_PROP_FRAME_HEIGHT);
    cout << "Frame Size = " << dWidth << "x" << dHeight << endl;

    Size frameSize(static_cast<int>(dWidth), static_cast<int>(dHeight));
    //initialize the VideoWriter object 
    //VideoWriter oVideoWriter("LaneDetection.avi", CV_FOURCC('P','I','M','1'), 20, frameSize, true); 

    // BIG While !!
    while(1){
        capture >> frame;
        if (frame.empty())
            break;
        imshow(window_name, frame);
        //delay N millis, usually long enough to display and capture input
        char key = (char)waitKey(1); 
        switch (key) {
		    case 'q':
		    case 'Q':
		    case 27: //escape key
		        return 0;
            case 32://space key : Pause
                while(waitKey(0) != 32);
                break;
		    case 13: //Enter key : Save an image
		        sprintf(filename,"filename%.3d.jpg",n++);
		        imwrite(filename,frame);
		        cout << "Saved " << filename << endl;
		        break;
		    default:
		        break;
        }
//*************************************** 主功能程序从此开始 *******************************************//
        // set the ROI for the frame and convert color to GRAY (Save time)
        Rect roi(0,frame.rows*0.65,frame.cols,frame.rows*0.35);
        Mat imgROI = frame(roi);
        cvtColor(imgROI,imgROI,CV_RGB2GRAY);
        //Mat ROI_gray;
        //Mat laneMask;
        //threshold(imgROI,laneMask,150,255,THRESH_BINARY);
        //bitwise_and(imgROI,laneMask,imgROI);
        imshow("ROI Image",imgROI);

        Mat PyrDown;
        pyrDown(imgROI,imgROI);
        //pyrDown(imgROI,imgROI);
        //imshow("PyrDown Image",PyrDown);

        //equalizeHist( imgROI, imgROI );
        //imshow("equalizeHist",imgROI);

        Mat contours;
        Canny(imgROI,contours,100,200);
        Mat contoursInv;
        threshold(contours,contoursInv,128,255,THRESH_BINARY_INV);
        imshow("Contours",contoursInv);

        // Create LineFinder instance
        LineFinder ld;

        // Set probabilistic Hough parameters
        ld.setLineLengthAndGap(0.4*contours.rows,0.1*contours.rows);
        ld.setMinVote(20);
        ld.setShift(3);

        // Detect lines
        Mat houghP(imgROI.rows*2,imgROI.cols*2,CV_8U,Scalar(255));
        std::vector<cv::Vec4i> li= ld.findLines(contours);
        ld.drawDetectedLines(houghP);
        imshow("Detected Lines with HoughP",houghP);

        //pyrUp(houghP,houghP);
        //imshow("Lane",houghP);

    }
    return 0;
}
