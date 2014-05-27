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
		double whitePointsRate(int row,int cb,int ce){
			uchar *data = image.ptr<uchar>(row);
			int cbegin;
			for(int c=cb;c<ce;c++){
				uchar tmp = data[c];
				if(tmp > 250){
					cbegin = c;
					break;
				}
			}

			if(cbegin==0)
				return -1;
			
			int cnt = 0;
			int gap = 5;
			double rate;
			for(int c=cbegin;c<ce;c++){
				uchar tmp = data[c];
				if(tmp == 255){
					cnt++;
					gap = 5;
				}
				else if(cnt>0 && gap>0)
					gap--;
			}
			printf("cnt: %d row:%d c: %d - %d\n",cnt,row,cb,ce);
			rate = cnt/double(ce-cb);
			return rate;
		}

		void vehiclesLocation(){
			cv::Point p1(0.29 * image.cols, 0.67 * image.rows);
			cv::Point p2(0.45 * image.cols, 0.077 * image.rows);
			cv::LineIterator it(image, p1, p2, 8);
			
			cv::Point p3(0.71 * image.cols, 0.67 * image.rows);
			cv::Point p4(0.55 * image.cols, 0.077 * image.rows);
			cv::LineIterator it2(image, p3, p4, 8);
			

			int row;
			int cb;
			int ce;
			double max_rate;
			int max_row;
			//printf("%d\n",it.count);
			for(int i = 0; i < it.count; i++, it2++,it++)
			{
				(*it)[0] = 255;
				(*it2)[0] = 255;
				cb = (it.pos()).x;			
				ce = (it2.pos()).x;
				row = (it.pos()).y;
				//cv::LineIterator it_tmp = it++;
				// if(row == (it_tmp.pos()).y )
				// 	continue;
				

				printf("%d: %d,%d \n",row,cb+2,ce-2);

				//for(int r= 20 ;r<(image.rows-30);r++){
					double rate = whitePointsRate(row,cb+2,ce-2);
					if(rate > max_rate){
						max_rate = rate ;
						max_row = row;
						printf("%f\n",max_rate);
					}
				//}			
			}
			printf("Max: %d %f\n",max_row,max_rate);	

		}



};

#endif