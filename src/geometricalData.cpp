#include "geometricalData.h"

geometricalData::geometricalData(){}

geometricalData::~geometricalData(){}


double geometricalData::getMedian(std::vector<double> vec) {

	double median = 0;
	int vecLen = vec.size();
	std::sort(vec.begin(), vec.end());

	if (vecLen % 2 > 0)
	{
		median = (vec[(vecLen+1)/2] + vec[(vecLen-1)/2])/2;	
	}
	else
	{
  		median = vec[vecLen/2];
	}
	return median;
}

float geometricalData::getSquareArea_(cv::Point2f p1, cv::Point2f p2, cv::Point2f p3) {

	return fabs((p1.x*(p2.y-p3.y) + p2.x*(p3.y-p1.y) + p3.x*(p1.y-p2.y)));
}

bool geometricalData::inObject(cv::Point2f *corner, cv::Point2f point) {

	float A = getSquareArea_(corner[0], corner[1], corner[2]);
	float A1 = getSquareArea_(point, corner[1], corner[2]);
	float A2 = getSquareArea_(corner[0], point, corner[2]);
	float A3 = getSquareArea_(corner[0], corner[1], point);

	return (int(A) == int(A1+A2+A3));
}



double geometricalData::getHeight(cv::Mat &xyz, cv::Mat &src, cv::Point2f *corners, double conf_dist) {

	confDist_ = conf_dist;

	int whitePixels = 0, totalPixels = 0;
	for (int x = 0; x < xyz.cols; x++) 
  	{
		for (int y = 0; y < xyz.rows; y++) 
		{	
			if (inObject(corners, cv::Point2f(x,y)))
			{
				if ((conf_dist - xyz.at<cv::Vec3s>(cv::Point(x, y))[0]) > 5)
				{
					vecdist_.push_back(conf_dist - xyz.at<cv::Vec3s>(cv::Point(x, y))[0]);
				}

				if (src.at<uchar>(cv::Point(x, y)) > 0)
				{
					whitePixels++;
				}
				totalPixels++;
			}
		}
	}

	pixelRatio_ = double(whitePixels)/totalPixels*100;
	std::cout << "pixelRatio: " << pixelRatio_ << "%" << std::endl;

	H_ = getMedian(vecdist_) +5; // +5 = the offset placed on the config dist value
	return H_;
}


void geometricalData::setPixelLength(cv::Mat &xyz, cv::Mat &src, cv::Point2f *corners, double conf_dist) {

	double H = getHeight(xyz, src, corners, conf_dist);

	float pixelArea = 0;

	if (H < 0)
	{
		H = 0;
	}

	if (pixelRatio_ > 75)
	{
		pixelArea = (A_ + B_ * (confDist_ - H) + float(C_ * pow((confDist_ - H), 2))) / double(numOfPixels_); // pixelArea = (A + Bx + Cx²) / numOfPixels
	}
	else
	{	
		std::vector<double> temp_vecdist;
		sort(vecdist_.begin(), vecdist_.end());
		for (int i = (vecdist_.size()-50); i < vecdist_.size(); ++i)
		{
			temp_vecdist.push_back(vecdist_[i]);
		}
		pixelArea = (A_ + B_ * (confDist_ - H/2) + float(C_ * pow((confDist_ - H/2), 2))) / double(numOfPixels_); // pixelArea = (A + Bx + Cx²) / numOfPixels
	}
	pixelLength_ = sqrt(pixelArea);
}


double* geometricalData::getLengthAndWidth(double *LW, cv::Point2f *corners, double pixelLength) {

	double hyp[3], hos = 0, mod = 0;
	for (int i = 0; i < 3; i++)
	{
		mod = int(corners[1 + i].y) - int(corners[0 + i].y) ;
		hos = int(corners[1 + i].x) - int(corners[0 + i].x) ;
		hyp[i] = sqrt(pow(mod, 2) + pow(hos, 2)) -2;	// -2 = boundary box offset
	}
	int hypLen = sizeof(hyp)/sizeof(*hyp);
	std::sort(hyp, hyp + hypLen);

	if (hyp[0] > hyp[2])
	{
		LW[0] = hyp[0] * pixelLength; // length
		LW[1] = hyp[2] * pixelLength; // width
	}
	else
	{
		LW[0] = hyp[2] * pixelLength; // length
		LW[1] = hyp[0] * pixelLength; // width
	}

	return LW;
}

double* geometricalData::getDimensions(cv::Mat &xyz, cv::Mat &src, double *LWH, cv::Point2f *corners, double conf_dist) {

	setPixelLength(xyz, src, corners, conf_dist);

	double LW[2];
	double* L_W = getLengthAndWidth(LW, corners, pixelLength_);

	LWH[0] = L_W[0];
	LWH[1] = L_W[1];
	LWH[2] = H_;

	return LWH;
}