#if !defined CARF
#define CARF

using namespace cv; 

class CarFinder{

	private:

		Mat image;

		Mat erode;

		Mat structure;

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

	public:

		CarFinder() : erode( getStructuringElement(MORPH_RECT,Size(10,1),Point(0,0)) ),
					  structure( getStructuringElement(MORPH_RECT,Size(5,1),Point(-1,-1)) )
		{}

		void setImage(Mat img){
			image = img.clone();
		}

		Mat getImage(){ return image;}

		void preProcess(){
			max_threshold(shadowBound(image),255);
			max_threshold(shadowBound2(image),255);
		}










};

#endif