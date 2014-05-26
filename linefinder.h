#if !defined LINEF
#define LINEF

#define PI 3.1415926

class LineFinder {

  private:

	  // original image
	  cv::Mat img;

	  // vector containing the end points 
	  // of the detected lines
	  std::vector<cv::Vec4i> lines;

	  std::vector<cv::Vec4d> frame;

	  // accumulator resolution parameters
	  double deltaRho;
	  double deltaTheta;

	  // minimum number of votes that a line 
	  // must receive before being considered
	  int minVote;

	  // min length for a line
	  double minLength;

	  // max allowed gap along the line
	  double maxGap;

	  // distance to shift the drawn lines down when using a ROI
	  int shift;

	  // the thickness of detected lane
	  int laneThickness;

  public:

	  // Default accumulator resolution is 1 pixel by 1 degree
	  // no gap, no mimimum length
	  LineFinder() : deltaRho(1), deltaTheta(PI/180), minVote(10), minLength(0.), maxGap(0.) {}

	  //设置img，方便后面调用(如findLine)
	  void setImg(cv::Mat image){
	  	  img = image;
	  }

	  // Set the resolution of the accumulator
	  void setAccResolution(double dRho, double dTheta) {
		  deltaRho= dRho;
		  deltaTheta= dTheta;
	  }

	  // Set the minimum number of votes
	  void setMinVote(int minv) {

		  minVote= minv;
	  }

	  // Set line length and gap
	  void setLineLengthAndGap(double length, double gap) {

		  minLength= length;
		  maxGap= gap;
	  }

	  // set image shift
	  void setShift(int imgShift) {

		  shift = imgShift;
	  }

	  // set lane thickness
	  void setThick(int thick){

	  	  laneThickness = thick;
	  }
	  // get the line gradient 取得斜率
	  double getK(std::vector<cv::Vec4i>::iterator &it){

		cv::Point pt1((*it)[0],(*it)[1]+shift); 
	  	cv::Point pt2((*it)[2],(*it)[3]+shift);
	  	double k = (pt2.x-pt1.x)/(pt1.y-pt2.y+0.0001);
	  	return k;
	  }

	  // 取得角度，不同于常规（懒得再改），具体见图片
	  double getTheta(double k){
	  	return atan(k)*90;
	  }

	  // Apply probabilistic Hough Transform And filter lines by limted theta
	  std::vector<cv::Vec4i> findLines(cv::Mat& binary) {

	  		lines.clear();
		    cv::HoughLinesP(binary,lines,deltaRho,deltaTheta,minVote, minLength, maxGap);

	  		std::vector<cv::Vec4i>::iterator it = lines.begin();
	  		std::vector<cv::Vec4i> filter_lines;

			//极角约束Hough变换 不在theta范围内的不压入栈
	  		while (it!=lines.end()) {
		
			  double k = fabs(getK(it));
			  double theta = getTheta(k);
			  //if(k>0.8 && k<5.5)
			  if(theta>80 && theta<130)
			  	filter_lines.push_back(*it);
			  ++it;
			}

			// 因为随机Hough变换，无法确定(it[0],it[1]),(it[2],it[3])哪个点在上，哪个在下，故通过循环使(it[0],it[1])在上
			// 具体见图
			std::vector<cv::Vec4i>::iterator it1 = filter_lines.begin();
			std::vector<cv::Vec4i>::iterator it2 = filter_lines.begin();

			for (it1 = filter_lines.begin();it1 != filter_lines.end();it1++) {
				if((*it1)[1] < (*it1)[3]){
					int tmp = (*it1)[1];
					(*it1)[1] = (*it1)[3];
					(*it1)[3] = tmp;

					tmp = (*it1)[0];
					(*it1)[0] = (*it1)[2];
					(*it1)[2] = tmp;
				}
			}

			// 过滤角度相差不大的线
			it1 = filter_lines.begin();
			while (it1 != filter_lines.end()) {
				double k1 = getK(it1);
				double t1 = getTheta(k1);

				it2 = it1 + 1;
				while (it2 != filter_lines.end()){
					double k2 = getK(it2);
					double t2 = getTheta(k2);
					//除去斜率相近的邻线  应相差多少才合适呢?
					if(t1*t2 >0 && fabs(t1-t2) < 10 ){
						//取最上的一条线，不要让其不断跳变
						if((*it2)[0] < (*it1)[0]){
							cv::Vec<int,4> tmp = *it2;
							*it2 = *it1;
							*it1 = tmp;
						}
						// 除去冗余的线
						it2 = filter_lines.erase(it2);
					}
					else
						it2++;
				}
				it1++;
			}

			// 过滤错乱的线
			it1 = filter_lines.begin();
			while(it1 != filter_lines.end()){
				double k1 = getK(it1);
				if(k1 < 0 && (*it1)[2] < img.cols/2)
				 	it1 = filter_lines.erase(it1);
					else if(k1 > 0 && (*it1)[0] > img.cols/2)
						it1 = filter_lines.erase(it1);
					else 
						it1++;
			}

			// 显示共检测到几条线
			std::cout<<"size:"<<filter_lines.size()<<std::endl;

			lines.clear();
			lines = filter_lines;

			return lines;
	  }

	  // void frameDifference(){
	  // 	std::vector<cv::Vec4d> Currentframe;
	  //   std::vector<cv::Vec4d>::iterator it_frame = Currentframe.begin();

  	// 	std::vector<cv::Vec4i>::iterator it_line = lines.begin();

  	// 	Currentframe.push_back({0.1,0,0,0});
  	// 	(*it_frame)[0] = 150.2;
  	// 	(*it_frame)[1] = 150.2;
  	// 	(*it_frame)[2] = 150.2;
  	// 	(*it_frame)[3] = 150.2;

  		// for(int i=0;i<lines.size();i++){
  		//   double k = getK(it_line);
		  // double t = getTheta(k);
		  // (*it_frame)[i] = t; 
		  // ++it_line;
  		// }
  // 		while (it_line!=lines.end()) {	
		//   double k = getK(it_line);
		//   double t = getTheta(k);
		//   int i = 0;
		//   if()
		//   (*it_frame)[i] = t; 
		//   ++i;
		//   ++it_line;
		// }

		// if(frame.size()<5){
		// 	frame.push_back(*it_frame);
		// }
		// else{
		// 	frame.erase(frame.begin());
		// 	frame.push_back(*it_frame);
		// }

		// it_frame = frame.begin();
		// std::cout<<"Last frame: "<<(*it_frame)[0]<<(*it_frame)[1]<<std::endl;


	  // }

	  // Draw the detected lines on an image
	  void drawDetectedLines(cv::Mat &image, cv::Scalar color=cv::Scalar(0,0,0)) {
		
		  // Draw the lines
		  std::vector<cv::Vec4i>::iterator it2= lines.begin();
	
		  while (it2!=lines.end()) {

			  double k = getK(it2);
			  //若k<0，蓝色 则线在左上，需往下移；若k>0，黄色 则线在右下，需往上移  具体见图
			  if(k<0){
			  	cv::Point pt2((*it2)[0]*2,(*it2)[1]*2-shift);
			  	cv::Point pt1((*it2)[2]*2,(*it2)[3]*2-shift);
			  	cv::Point ptTop((pt2.x-pt1.x)/(pt1.y-pt2.y + 0.0001)*pt2.y + pt2.x , 0);
			  	cv::Point ptLow(image.cols,(pt2.y-pt1.y)/(pt2.x-pt1.x + 0.0001)*(image.cols - pt1.x) + pt1.y);
			  	cv::line( image, ptTop, ptLow, color, laneThickness );
			  	cv::line( image, pt1, pt2, cv::Scalar(0,200,200), laneThickness );
			  	std::cout<<"k:"<<k<<" angle:"<<getTheta(k);
				std::cout << "("<< pt1 <<"," << pt2 << ")\n"; 
			  }
			  else{
			  	cv::Point pt2((*it2)[0]*2,(*it2)[1]*2+shift);
			  	cv::Point pt1((*it2)[2]*2,(*it2)[3]*2+shift);
			  	cv::Point ptTop((pt2.x-pt1.x)/(pt1.y-pt2.y + 0.0001)*pt2.y + pt2.x , 0);
			  	cv::Point ptLow(0,(pt2.y-pt1.y)/(pt1.x-pt2.x + 0.0001)*pt1.x + pt1.y);
			  	cv::line( image, ptTop, ptLow, color, laneThickness );
			  	cv::line( image, pt1, pt2, cv::Scalar(200,200,0), laneThickness);
			  	std::cout<<"k:"<<k<<" angle:"<<getTheta(k);
				std::cout << "("<< pt1 <<"," << pt2 << ")\n"; 
			  }       
			  ++it2;	
		  }
		  // frameDifference();
		  std::cout<<"\n";
	  }

	// 画虚线(p1:起点 p2:终点)
	void drawDashLine(cv::Mat img ,cv::Point p1 , cv::Point p2){
		cv::LineIterator it(img, p1, p2, 8);            // get a line iterator
		for(int i = 0; i < it.count; i++,it++)
		    if ( i % 3 !=0 ){
		    	(*it)[1] = 255;         // every 5'th pixel gets dropped, blue stipple line
		    	(*it)[2] = 255;
		    }
	}

	// 导航框初始化，方便调用
	void dashLine_init(cv::Mat img){
        drawDashLine(img, 
                     cv::Point(0.07 * img.cols, img.rows),
                     cv::Point(0.45 * img.cols, 0.64 * img.rows)
                     );
        drawDashLine(img, 
                     cv::Point(0.45 * img.cols, 0.64 * img.rows),
                     cv::Point(0.45 * img.cols, 0.60 * img.rows)
                     );
        drawDashLine(img, 
                     cv::Point(0.93 * img.cols, img.rows),
                     cv::Point(0.55 * img.cols, 0.64 * img.rows)
                     );
        drawDashLine(img, 
                     cv::Point(0.55 * img.cols, 0.64 * img.rows),
                     cv::Point(0.55 * img.cols, 0.60 * img.rows)
                     );
    }
};


#endif
