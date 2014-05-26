#if !defined CARF
#define CARF

using namespace cv; 

class CarFinder{

	private:

		Mat image;

		Mat erode_structrue;

		Mat morph_structure;

		struct vehicle{
			int r;          // row 第几行
			int vb;			// vb  起始列
			int ve;			// ve  终止列
		};

		struct vehicleBox{
			bool valid;
			CvPoint bmin;
			CvPoint bmax;
			unsigned int width;
			unsigned int height;
		};

		//	遍历整幅image图像，对大于threshold的图像点，将max_value赋给图像，否则保持图像原值
		int max_threshold(double threshold,uchar max_value){
			for(int r=0;r < image.rows;r++){
				uchar *data = image.ptr<uchar>(r);
				for(int c=0;c < image.cols;c++)
					if(data[c] > threshold)
						data[c] = max_value;
			}
			return 0;
		}

		//车的阴影第一次阈值分割 shadowBound
		double shadowBound(Mat img ){
			Scalar mean;
			Scalar dev;

			meanStdDev(img,mean,dev);

			double a = mean.val[0]/dev.val[0];
			double thresh_1 = mean.val[0]-dev.val[0]/a;
			return (thresh_1);
		}

		//车的阴影第二次阈值分割，在低于thresh_1的像素点中继续提取
		double shadowBound2(Mat img){
			Mat mask = img.clone();

			threshold(img,mask,254,255,THRESH_TOZERO_INV);

			Scalar mean;
			Scalar dev;
			meanStdDev(img,mean,dev,mask);

			double b = mean.val[0]/dev.val[0];
			double thresh_2 = mean.val[0]-dev.val[0]/b;

			return (thresh_2);
		}

		// 功能： 对roi进行sobel变换，检测边缘后保存至roi_sobel
		int sobel(){
			Mat temp(image.size(),CV_16S,1);
			Sobel(image,temp,CV_16S,0,1,3);
			convertScaleAbs(temp,temp);
			threshold(temp,image,0,255,THRESH_OTSU);
			return 0;
		}

	public:

		CarFinder() : erode_structrue( getStructuringElement(MORPH_RECT,Size(10,1),Point(0,0)) ),
					  morph_structure( getStructuringElement(MORPH_RECT,Size(5,1),Point(-1,-1)) )
		{}

		void setImage(Mat img){
			image = img.clone();
		}

		Mat getImage(){ return image;}

		//预处理函数：含两次阈值分割，腐蚀，形态学运算
		void preProcess(){
			max_threshold(shadowBound(image),255);
			max_threshold(shadowBound2(image),255);
			sobel();
			cv::erode(image,image,erode_structrue,Point(0,0));
			morphologyEx(image,image,MORPH_CLOSE,morph_structure);
		}

		// 功能： 白线检测  从第row行开始，检测cb列－ce列的白点率
		// 	  从cb开始检测，右亮度大于250则标记为cbegin,然后在cbegin
		//    之后大于250的点，让cnt自加，最后用cnt/(ce-cb)算出白点率	
		// double whitePointsRate( int row,int cb,int ce){
		// 	Mat img = image.clone();
		// 	uchar *data = img.ptr<uchar>(row);
		// 	int cbegin = 0;
		// 	for(int c=cb;c<ce;c++){
		// 		//std::cout<<ptr[c]<<std::endl;
		// 		uchar tmp = data[c];
		// 		if(tmp > 250){
		// 			cbegin = c;
		// 			break;
		// 		}
		// 	}

		// 	if(cbegin == 0){
		// 		return -1;
		// 	}

		// 	int cnt = 5;
		// 	int gap = 5;
		// 	double rate;
		// 	for(int c=cbegin;c<ce || gap < 0;c++){
		// 		uchar tmp = data[c];
		// 		if(tmp > 250){
		// 			cnt++;
		// 			gap = 5;
		// 		}
		// 		else if(cnt>0 && gap>0)
		// 			gap--;
		// 	}
		// 	rate = cnt/((ce-cb)*0.0001);
		// 	return rate;
		// }

		// double whitePointsRate(){
		// 	std::cout<<image.cols<<"X"<<image.rows<<" type:"<<image.type()<<std::endl;
		// 	Mat img = image.clone();
		// 	for(int j=0;j<21;j++){
		// 		uchar *data = img.ptr<uchar>(j);
		// 		for(int i=20;i<img.cols;i++){
		// 			// if(data[i].val > 250)
		// 			// 	std::cout<<data[i].val;
		// 			//Scalar intensity = img.at<uchar>(y, x);
		// 			uchar tmp = data[i];
		// 			//std::cout<<tmp;
		// 			//printf("%d\n",tmp );
		// 			if(tmp > 250)
		// 				data[i] = 127;

		// 		}
		// 	}
		// 	imshow("test",img);

		// }

		// 功能： 白线检测  从第row行开始，检测cb列－ce列的白点率
		// 	  从cb开始检测，右亮度大于250则标记为cbegin,然后在cbegin
		//    之后大于250的点，让cnt自加，最后用cnt/(ce-cb)算出白点率	
		double whitePointsRate(int row,int cb,int ce){
			uchar *data = image.ptr<uchar>(row);
			int cbegin;
			for(int c=cb;c<ce;c++){
				uchar tmp = data[c];
				if(tmp > 250){
					cbegin = c;
					//printf("%d\n",cbegin);
					break;
					//return cbegin;
				}
			}
			//printf("%d\n",cbegin);

			int cnt = 5;
			int gap = 5;
			double rate;
			for(int c=cbegin;c<ce;c++){
				uchar tmp = data[c];
				if(tmp > 250){
					cnt++;
					gap = 5;
				}
				else if(cnt>0 && gap>0)
					gap--;
			}
			rate = cnt/((ce-cb)+0.0001);
			//printf("%f\n",rate);
			return rate;

		}

		void vehiclesLocation(){
			for(int r= 20 ;r<(image.rows-30);r++){
				double rate = whitePointsRate(r,20,image.cols-20);
				if(rate <= 1){
					printf("%f\n",rate);
				}
			}
		}



};

#endif