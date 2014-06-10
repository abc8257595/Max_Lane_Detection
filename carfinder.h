#if !defined CARF
#define CARF

using namespace cv; 

class CarFinder{

	private:

		// 每帧图像都会传入到此
		Mat image;

		// 自定义的腐蚀结构
		Mat erode_structrue;

		// 自定义的形态学变换结构
		Mat morph_structure;

		// 经过计算后得出的最大白线率所在的行、起始列、终止列
		int max_cend , max_cbegin ,max_row;

		double max_rate;

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

		// 对象初始化
		CarFinder() : erode_structrue( getStructuringElement(MORPH_RECT,Size(10,1),Point(0,0)) ),
					  morph_structure( getStructuringElement(MORPH_RECT,Size(5,1),Point(-1,-1)) )
		{}

		// 设置图像image
		void setImage(Mat img){
			image = img.clone();
		}

		// 获取处理后的图像，供外部调用
		Mat getImage(){ return image;}

		//预处理函数：含两次阈值分割，腐蚀，形态学运算
		void preProcess(){
			max_threshold(shadowBound(image),255);
			max_threshold(shadowBound2(image),255);
			sobel();
			cv::erode(image,image,erode_structrue,Point(0,0));
			morphologyEx(image,image,MORPH_CLOSE,morph_structure);
		}

		// 功能： 计算白点率  从第row行开始，检测cb列－ce列的白点率
		// 	  从cb开始检测，右亮度大于250则标记为cbegin,然后在cbegin
		//    之后大于250的点，让cnt自加，最后用cnt/(ce-cb)算出白点率
		//    cbegin传出起始列，cend传出终止列
		double whitePointsRate(int row,int cb,int ce,int &cbegin,int &cend){
			// 遍历指定行、列
			uchar *data = image.ptr<uchar>(row);
			for(int c=cb;c<ce;c++){
				uchar tmp = data[c];
				if(tmp > 250){
					cbegin = c;
					break;
				}
			}

			// cbegin == 0 则认为出错
			if(cbegin==0)
				return -1;
			
			// cnt白点计数器，gap可容忍的白点间断数，rate白点占ce-cb的比率，c循环计数(需传出数据)
			int cnt = 0;
			int gap = 3;
			double rate;
			int c ;
			// 计算cb到ce的白点率
			for(c=cbegin;c<ce && gap > 0;c++){
				uchar tmp = data[c];
				if(tmp == 255){
					cnt++;
					gap = 3;
				}
				else if(cnt>0 && gap>0)
					gap--;
			}
			// 传出cend的数值
			cend = c - 2;

			// 返回白点率
			rate = cnt/double(ce-cb);
			return rate;
		}

		// 计算出最大白点率的行，起始列，终止列
		void vehiclesLocation(){
			// 缩小后的图像尺寸 295 X 93 
			// 根据前面画的虚线重映射的点，数字来源计算如下：
			// p1(86,63) :0.29 ＝ 86 / 295; 0.67 = 63 / 93
			// p2: 0.45 根据linefinder虚线定义的0.45； 0.077 ＝ 1 - 0.36/0.39 
			cv::Point p1(0.29 * image.cols, 0.67 * image.rows);
			cv::Point p2(0.45 * image.cols, 0.077 * image.rows);
			cv::LineIterator it(image, p1, p2, 8);
			
			// 对称的，y不变，x取补
			cv::Point p3(0.71 * image.cols, 0.67 * image.rows);
			cv::Point p4(0.55 * image.cols, 0.077 * image.rows);

			// 线上的点遍历
			cv::LineIterator it2(image, p3, p4, 8);
			
			int row, cb , ce ;
			double max_rate_tmp;

			for(int i = 0; i < it.count; i++, it2++,it++){

				// 在小图上画虚线
				(*it)[0] = 255;
				(*it2)[0] = 255;

				// 取得线上的行数，起始列，终止列
				cb = (it.pos()).x;			
				ce = (it2.pos()).x;
				row = (it.pos()).y;
				
				// 定义cbegin,cend以取出起始列，终止列，一定要赋初值
				int cbegin = 0;
				int cend = 0;

				// 调用计算白点率
				double rate = whitePointsRate(row,cb+2,ce-2,cbegin,cend);
				// 取出白点率最大的行，起始列，终止列
				if(rate > max_rate){
					max_rate_tmp = rate ;
					max_row = row;
					max_cbegin = cbegin;
					max_cend = cend;
				}
			}
			max_rate = max_rate_tmp;
			//printf("Max: %d %d %d %f\n",max_row,max_cbegin,max_cend,max_rate);	
		}

		// 取得车框左下方点，可自己调整
		cv::Point getPt1(){
			return Point(max_cbegin - 10 ,max_row + 5 );
		}

		// 取得车框右上方点，可自己调整
		cv::Point getPt2(){
			return Point(max_cend + 10,max_row - 15);
		}
};

#endif