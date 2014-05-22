/*------------------------------------------------------------------------------------------*\


\*------------------------------------------------------------------------------------------*/

#include <highgui/highgui.hpp>
#include <core/core.hpp>
#include <imgproc/imgproc.hpp>
#include <iostream>
#include <stdio.h>
#include <vector>
#include "linefinder.h"
#include "carfinder.h"


#define uchar unsigned char

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
    double dFps = capture.get(CV_CAP_PROP_FPS);
    cout << "Frame Size = " << dWidth << "x" << dHeight << endl;
    cout << "FPS :" << dFps << endl;

    Size frameSize(static_cast<int>(dWidth), static_cast<int>(dHeight)); 

    // BIG While !!
    while(1){
        capture >> frame;
        if (frame.empty())
            break;
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
        // set the ROI for the frame and convert color to GRAY (Save time)  +1: match the houghP
        Rect roi(0,frame.rows*0.65,frame.cols,frame.rows*0.35 + 1);
        Mat imgROI_color = frame(roi);
        Mat imgROI_grey;
        cvtColor(imgROI_color,imgROI_grey,CV_RGB2GRAY);
        imshow("ROI Image",imgROI_grey);

        //缩小图像以减少计算量
        Mat imgROI_grey_down;
        pyrDown(imgROI_grey,imgROI_grey_down);

        //本意是想增加对比度，更好地显示出白线、黄线，效果并不理想
        // for(int r=0;r < imgROI_grey_down.rows;r++){
        //     uchar *data = imgROI_grey_down.ptr<uchar>(r);
        //     for(int c=0;c < imgROI_grey_down.cols;c++)
        //             data[c] = saturate_cast<uchar>(data[c] * data[c] / 100);
        // }
        //imshow("bringt",imgROI_grey_down);

        //腐蚀以细化车道线，想通过这个方法更好地拟合出车道线 在缩小的图里容易直接把线腐蚀没，最后居然是原来的最好！
        // Mat erosion_dst;
        // uchar erosion_size = 1 ;
        // Mat element = getStructuringElement( MORPH_CROSS,
        //                                Size( 2*erosion_size + 1, 2*erosion_size+1 ),
        //                                Point( erosion_size, erosion_size ) );
        // erode( imgROI_grey_down, erosion_dst, element , Point(-1,-1) , 1 );
        // imshow("erosion",erosion_dst);

        //Canny算法检出边缘，但100，200的阀值怎么取才合理是个问题 
        Mat contours;
        Canny(imgROI_grey_down,contours,100,200);
        Mat contoursInv;
        threshold(contours,contoursInv,128,255,THRESH_BINARY_INV);
        imshow("Contours",contoursInv);

        // Create LineFinder instance
        LineFinder ld;

        // Set probabilistic Hough parameters , set drawing line's thickness
        ld.setLineLengthAndGap(0.4*contours.rows,0.1*contours.rows);
        ld.setMinVote(20);
        ld.setShift(0);
        ld.setThick(3);

        // Detect lines 检测直线，并显示
        Mat houghP(imgROI_grey_down.rows*2,imgROI_grey_down.cols*2,CV_8UC3,cv::Scalar(0,0,0));
        std::vector<cv::Vec4i> li= ld.findLines(contours);
        ld.drawDetectedLines(houghP,cv::Scalar(0,0,255));
        imshow("Detected Lines with HoughP",houghP);

        // Display on the real-time frame
        cv::addWeighted(imgROI_color,0.7,houghP,1.0,0.,imgROI_color);
        imshow(window_name, frame);

    /*********************************** car detection ***********************************************/
        
        CarFinder cf;

        cf.setImage(imgROI_grey_down);
        cf.preProcess();
        imshow("preProcess",cf.getImage());

    }
    return 0;
}
