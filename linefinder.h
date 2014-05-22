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

	  // Apply probabilistic Hough Transform And filter lines by limted theta
	  std::vector<cv::Vec4i> findLines(cv::Mat& binary) {

	  		lines.clear();
		    cv::HoughLinesP(binary,lines,deltaRho,deltaTheta,minVote, minLength, maxGap);

	  		std::vector<cv::Vec4i>::iterator it = lines.begin();
	  		std::vector<cv::Vec4i> filter_lines;

			//极角约束Hough变换 不在k范围内的不压入栈
	  		while (it!=lines.end()) {
		
			  double k = fabs(getK(it));
			  if(k>0.3 && k<8.00)
			  	filter_lines.push_back(*it);
			  ++it;
			}

			// 因为随机Hough变换，无法确定(it[0],it[1]),(it[2],it[3])哪个点在上，哪个在下，故通过循环使(it[0],it[1])在下
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

			it1 = filter_lines.begin();
			while (it1 != filter_lines.end()) {
				double k1 = getK(it1);

				it2 = it1 + 1;
				while (it2 != filter_lines.end()){
					double k2 = getK(it2);
					//除去斜率相近的邻线  应相差多少才合适呢?
					if(k1*k2 >0 && fabs(k1-k2) < 1 ){
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

			lines.clear();
			lines = filter_lines;

			return lines;
	  }

	  // Draw the detected lines on an image
	  void drawDetectedLines(cv::Mat &image, cv::Scalar color=cv::Scalar(0,0,0)) {
		
		  // Draw the lines
		  std::vector<cv::Vec4i>::iterator it2= lines.begin();
	
		  while (it2!=lines.end()) {

			  double k = getK(it2);
			  //若k<0，则线在左上，需往下移；若k>0，则线在右下，需往上移
			  if(k<0){
			  	cv::Point pt2((*it2)[0]*2,(*it2)[1]*2-shift);
			  	cv::Point pt1((*it2)[2]*2,(*it2)[3]*2-shift);
			  	cv::Point ptTop((pt2.x-pt1.x)/(pt1.y-pt2.y + 0.0001)*pt2.y + pt2.x , 0);
			  	cv::Point ptLow(image.cols,(pt2.y-pt1.y)/(pt2.x-pt1.x + 0.0001)*(image.cols - pt1.x) + pt1.y);
			  	cv::line( image, ptTop, ptLow, color, laneThickness );
			  	cv::line( image, pt1, pt2, cv::Scalar(0,200,200), laneThickness );
			  }
			  else{
			  	cv::Point pt2((*it2)[0]*2,(*it2)[1]*2+shift);
			  	cv::Point pt1((*it2)[2]*2,(*it2)[3]*2+shift);
			  	cv::Point ptTop((pt2.x-pt1.x)/(pt1.y-pt2.y + 0.0001)*pt2.y + pt2.x , 0);
			  	cv::Point ptLow(0,(pt2.y-pt1.y)/(pt1.x-pt2.x + 0.0001)*pt1.x + pt1.y);
			  	cv::line( image, ptTop, ptLow, color, laneThickness );
			  	cv::line( image, pt1, pt2, cv::Scalar(200,200,0), laneThickness);
			  }       
			  
		//std::cout << " HoughP line: ("<< pt1 <<"," << pt2 << ")\n"; 
			  ++it2;	
		  }
	  }

};


#endif
