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
        imshow("pyrDown_img",imgROI_grey_down);

        //Canny算法检出边缘，但100，200的阀值怎么取才合理是个问题 
        Mat contours;
        Canny(imgROI_grey_down,contours,100,200);
        Mat contoursInv;
        threshold(contours,contoursInv,128,255,THRESH_BINARY_INV);
        imshow("Contours",contoursInv);

        // Create LineFinder instance
        LineFinder ld;

        // Set ld Image, Set probabilistic Hough parameters , set drawing line's thickness
        ld.setImg(contours);
        ld.setLineLengthAndGap(0.4*contours.rows,0.1*contours.rows);
        ld.setMinVote(20);
        ld.setShift(0);
        ld.setThick(3);

        // Detect lines 检测直线，并显示
        Mat houghP(imgROI_grey_down.rows*2,imgROI_grey_down.cols*2,CV_8UC3,cv::Scalar(0,0,0));
        std::vector<cv::Vec4i> li= ld.findLines(contours);
        ld.drawDetectedLines(houghP,cv::Scalar(0,0,255));
        imshow("Detected Lines with HoughP",houghP);

        //显示默认虚线
        ld.dashLine_init(frame);    

        // Display on the real-time frame
        cv::addWeighted(imgROI_color,0.7,houghP,1.0,0.,imgROI_color);

    /*********************************** car detection ***********************************************/
        
        CarFinder cf;
        // 设置车辆检测ROI区域，与车道检测区域略有不同，所以重新定义大小
        Rect car_ROI(0,frame.rows*0.6,frame.cols,frame.rows*0.39 + 1);
        Mat car_ROI_color = frame(car_ROI);
        // 转为灰度图
        Mat car_ROI_grey;
        cvtColor(car_ROI_color,car_ROI_grey,CV_RGB2GRAY);
        // 金字塔缩小，以减小计算量
        Mat car_ROI_grey_down;
        pyrDown(car_ROI_grey,car_ROI_grey_down);

        // 传入car_ROI_grey_down图像 ,图像预处理
        cf.setImage(car_ROI_grey_down);
        cf.preProcess();
        imshow("car_shadow",cf.getImage());
        
        // 调用函数 检出车辆阴影位置，放在私有变量里
        cf.vehiclesLocation();

        // 画车框，将坐标放大两倍(因为之前的图像是缩小的)颜色为黄色，粗细为2
        rectangle(car_ROI_color, cf.getPt1()*2, cf.getPt2()*2 , Scalar( 0,255,255 ), 2, 8, 0);

        // 显示车道线，车检测区域虚线，前方车辆框
        imshow(window_name, frame);


    }
    return 0;
}
